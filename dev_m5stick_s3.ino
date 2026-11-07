/**
 * @file dev_m5stick_s3.ino
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief M5StickS3 評価F/W
 * @version 0.1
 * @date 2026-11-07
 * @copyright Copyright (c) 2026 Chimipupu All Rights Reserved.
 */

#include <M5Unified.h>
#include <M5GFX.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_freertos_hooks.h>
#include <driver/rtc_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// -----------------------------------------------------------
// #define M5PM1_L3B_USE
#ifdef M5PM1_L3B_USE
#include <M5PM1.h>
M5PM1    g_pm1;
static void _setup_pm1_l3b(void);
#endif

#define DEEP_SLEEP_TRG_TIME_MS (5000) /* n秒経過でDeepSleep */

#define PIN_BTN_SLEEP          (11)   /* 起床およびスリープ用ボタン (G11) */
#define PIN_BTN_RECORD         (12)   /* 録音操作用ボタン (G12) */

/* 音声サンプリング設定 (16kHz, 16bit モノラル: 30秒で 960,000 bytes) */
#define RECORD_SAMPLE_RATE     (16000)
#define RECORD_MAX_SECONDS     (30)
#define RECORD_TOTAL_SAMPLES   (RECORD_SAMPLE_RATE * RECORD_MAX_SECONDS)
#define RECORD_BUFFER_SIZE     (RECORD_TOTAL_SAMPLES * sizeof(int16_t))
#define AUDIO_CHUNK_SAMPLES    (512)  /* 1チャンクあたり32ms */

/* FFT設定 */
#define FFT_N                  (128)  /* 128点 FFT (ナイキスト 8kHz, 分解能 125Hz) */
#define FFT_BAR_COUNT          (16)   /* 表示バンド数 */

/* CPU負荷率計測周期 (ms) */
#define CPU_LOAD_PERIOD_MS     (500)

/* FreeRTOS メールボックス通信用構造体 */
typedef struct {
    uint8_t core0_load_pct;
    uint8_t core1_load_pct;
} cpu_load_msg_t;

typedef decltype(M5.Mic.config()) mic_cfg_t;

/* グローバル変数 (規約: g_, g_p_, g_is_) */
uint8_t*          g_p_record_buffer = NULL;
QueueHandle_t     g_p_cpu_mbox = NULL;
volatile uint32_t g_recorded_samples = 0;
uint32_t          g_step_count = 0;
float             g_velocity = 0.0f;
uint32_t          g_wake_start_ms = 0;
volatile uint32_t g_play_start_ms = 0;
volatile uint32_t g_total_play_sec = 0;
volatile uint32_t g_play_cur_sec = 0;
volatile float    g_fft_bars[FFT_BAR_COUNT] = {0.0f};
volatile bool     g_is_recording = false;
volatile bool     g_is_playing = false;
volatile bool     g_is_sleep_req = false;
M5Canvas          g_canvas(&M5.Display);

/* CPU Idle時間計測用 static変数 (規約: s_) */
static volatile uint32_t s_idle_us_core0 = 0;
static volatile uint32_t s_idle_us_core1 = 0;

/* プロトタイプ宣言 (規約: static関数は _func) */
static void _enter_deep_sleep(void);
static void _update_rotation(float gx, float gy, float gz, float ax, float ay);
static void _fft_radix2(float* p_real, float* p_imag, uint16_t n);
static void _calculate_fft_bars(const int16_t* p_samples, uint32_t sample_count, float* p_bars_out);
static void _draw_fft_bars(int32_t x, int32_t y, int32_t width, int32_t height, const volatile float* p_bars);
static bool _idle_hook_core0(void);
static bool _idle_hook_core1(void);
static void _task_cpu_load(void* p_arg);
static void _task_audio(void* p_arg);
static void _task_ui_motion(void* p_arg);

// -----------------------------------------------------------
// [Static 電源・画面制御]

#ifdef M5PM1_L3B_USE
/**
 * @brief M5PM1 PMICのL3B電源レールを有効化する
 */
static void _setup_pm1_l3b(void)
{
    g_pm1.begin(&M5.In_I2C, 0x6E, 400000);
    g_pm1.gpioSetOutput(M5PM1::GPIO2, HIGH);
}
#endif

/**
 * @brief ボタン解放を待機してから周辺機器を停止しDeepSleepへ移行
 */
static void _enter_deep_sleep(void)
{
    g_is_sleep_req = true;

    /* 残っている非同期SPI DMA転送があれば完了を待って画面消灯 */
    M5.Display.waitDMA();
    M5.Display.sleep();
    M5.Display.waitDisplay();

    /* 音声タスクのI2S動作停止を待機 */
    vTaskDelay(pdMS_TO_TICKS(50));
    M5.Speaker.stop();
    M5.Speaker.end();
    M5.Mic.end();

    /* G11ボタンから指が離れてHIGHになるまで待機 (即時再起床の防止) */
    while (digitalRead(PIN_BTN_SLEEP) == LOW)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /* チャタリング収束待ち */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* DeepSleep中もRTCドメインでG11のプルアップを維持 */
    rtc_gpio_pullup_en((gpio_num_t)PIN_BTN_SLEEP);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BTN_SLEEP);

    /* G11がLOWになったときに起床 */
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BTN_SLEEP, 0);
    esp_deep_sleep_start();
}

/**
 * @brief ジャイロおよび加速度センサの値からLCD表示方向を自動回転
 */
static void _update_rotation(float gx, float gy, float gz, float ax, float ay)
{
    int32_t        target_rotation;
    float          gyro_norm;
    static int32_t s_active_rotation = 0;

    target_rotation = s_active_rotation;
    gyro_norm = fabsf(gx) + fabsf(gy) + fabsf(gz);

    /* 激しい回転・振動中 (角速度大) は誤回転防止のため判定を保留 */
    if (gyro_norm < 120.0f) {
        if (fabsf(ax) > fabsf(ay)) {
            if (ax > 0.55f) {
                target_rotation = 1;
            } else if (ax < -0.55f) {
                target_rotation = 3;
            }
        } else {
            if (ay > 0.55f) {
                target_rotation = 0;
            } else if (ay < -0.55f) {
                target_rotation = 2;
            }
        }
    }

    if (target_rotation != s_active_rotation) {
        s_active_rotation = target_rotation;
        M5.Display.waitDMA();
        M5.Display.setRotation(s_active_rotation);
        g_canvas.deleteSprite();
        /* MALLOC_CAP_DMA (内部SRAM) に確保してゼロコピーSPI-DMA転送を有効化 */
        g_canvas.setPsram(false);
        g_canvas.createSprite(M5.Display.width(), M5.Display.height());
    }
}

// -----------------------------------------------------------
// [Static FFT処理]

/**
 * @brief FFT (基底2時間間引きCooley-Tukey FFT 128点)
 */
static void _fft_radix2(float* p_real, float* p_imag, uint16_t n)
{
    uint16_t i;
    uint16_t j;
    uint16_t k;
    uint16_t n1;
    uint16_t n2;
    float    c;
    float    s;
    float    t_real;
    float    t_imag;
    float    angle;

    j = 0;
    n2 = n / 2;
    for (i = 1; i < (n - 1); i++)
    {
        n1 = n2;
        while (j >= n1) {
            j -= n1;
            n1 /= 2;
        }
        j += n1;
        if (i < j) {
            t_real = p_real[i];
            p_real[i] = p_real[j];
            p_real[j] = t_real;

            t_imag = p_imag[i];
            p_imag[i] = p_imag[j];
            p_imag[j] = t_imag;
        }
    }

    n1 = 0;
    n2 = 1;
    for (i = 0; i < 7; i++)
    {
        n1 = n2;
        n2 += n2;
        for (j = 0; j < n1; j++)
        {
            angle = -3.14159265f * (float)j / (float)n1;
            c = cosf(angle);
            s = sinf(angle);
            for (k = j; k < n; k += n2)
            {
                t_real = (c * p_real[k + n1]) - (s * p_imag[k + n1]);
                t_imag = (s * p_real[k + n1]) + (c * p_imag[k + n1]);
                p_real[k + n1] = p_real[k] - t_real;
                p_imag[k + n1] = p_imag[k] - t_imag;
                p_real[k] += t_real;
                p_imag[k] += t_imag;
            }
        }
    }
}

/**
 * @brief FFTスペクトラムバーグラフの算出
 * @note 音声波形から窓関数適用・FFT計算を行い16バンドのバー比率(0.0〜1.0)を算出
 */
static void _calculate_fft_bars(const int16_t* p_samples, uint32_t sample_count, float* p_bars_out)
{
    static float s_real[FFT_N];
    static float s_imag[FFT_N];
    uint16_t     i;
    uint16_t     bar_idx;
    uint16_t     bin_idx;
    float        hann;
    float        mag;
    float        sum;
    float        scale;

    for (i = 0; i < FFT_N; i++)
    {
        if (i < sample_count) {
            hann = 0.5f * (1.0f - cosf(6.2831853f * (float)i / (float)(FFT_N - 1)));
            s_real[i] = ((float)p_samples[i] / 32768.0f) * hann;
        } else {
            s_real[i] = 0.0f;
        }
        s_imag[i] = 0.0f;
    }

    _fft_radix2(s_real, s_imag, FFT_N);

    for (bar_idx = 0; bar_idx < FFT_BAR_COUNT; bar_idx++)
    {
        sum = 0.0f;
        for (bin_idx = 0; bin_idx < 4; bin_idx++)
        {
            i = 1 + (bar_idx * 4) + bin_idx;
            if (i < (FFT_N / 2)) {
                mag = sqrtf((s_real[i] * s_real[i]) + (s_imag[i] * s_imag[i]));
                sum += mag;
            }
        }
        sum /= 4.0f;

        scale = sum * 14.0f;
        if (scale > 1.0f) {
            scale = 1.0f;
        } else if (scale < 0.0f) {
            scale = 0.0f;
        }
        p_bars_out[bar_idx] = scale;
    }
}

/**
 * @brief FFTスペクトラムバーグラフの描画
 */
static void _draw_fft_bars(int32_t x, int32_t y, int32_t width, int32_t height, const volatile float* p_bars)
{
    int32_t i;
    int32_t bar_w;
    int32_t bar_spacing;
    int32_t bar_h;
    int32_t bar_x;
    int32_t bar_y;

    bar_spacing = 2;
    bar_w = (width - (bar_spacing * (FFT_BAR_COUNT - 1))) / FFT_BAR_COUNT;
    if (bar_w < 1) {
        bar_w = 1;
    }

    for (i = 0; i < FFT_BAR_COUNT; i++)
    {
        bar_h = (int32_t)(p_bars[i] * (float)height);
        if (bar_h > height) {
            bar_h = height;
        }

        bar_x = x + (i * (bar_w + bar_spacing));
        bar_y = y + (height - bar_h);

        if (bar_h > 0) {
            g_canvas.fillRect(bar_x, bar_y, bar_w, bar_h, TFT_WHITE);
        }
    }
}

// -----------------------------------------------------------
// [CPU負荷計測用 Idle Hook & FreeRTOSタスク]

/**
 * @brief Core 0 Idle Hook (Idle中の微小経過時間を積算)
 */
static bool _idle_hook_core0(void)
{
    uint32_t        now_us;
    uint32_t        diff_us;
    static uint32_t s_last_us_core0 = 0;

    now_us = (uint32_t)esp_timer_get_time();
    diff_us = now_us - s_last_us_core0;
    s_last_us_core0 = now_us;

    if (diff_us < 50U) {
        s_idle_us_core0 += diff_us;
    }
    return false;
}

/**
 * @brief Core 1 Idle Hook (Idle中の微小経過時間を積算)
 */
static bool _idle_hook_core1(void)
{
    uint32_t        now_us;
    uint32_t        diff_us;
    static uint32_t s_last_us_core1 = 0;

    now_us = (uint32_t)esp_timer_get_time();
    diff_us = now_us - s_last_us_core1;
    s_last_us_core1 = now_us;

    if (diff_us < 50U) {
        s_idle_us_core1 += diff_us;
    }
    return false;
}

/**
 * @brief CPU負荷率計算タスク (FreeRTOSメールボックスでUIタスクへ送信)
 */
static void _task_cpu_load(void* p_arg)
{
    uint32_t        now_us;
    uint32_t        elapsed_us;
    uint32_t        idle0_us;
    uint32_t        idle1_us;
    uint32_t        idle0_pct;
    uint32_t        idle1_pct;
    cpu_load_msg_t  msg;
    static uint32_t s_prev_calc_us = 0;

    (void)p_arg;
    s_prev_calc_us = (uint32_t)esp_timer_get_time();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(CPU_LOAD_PERIOD_MS));

        now_us = (uint32_t)esp_timer_get_time();
        elapsed_us = now_us - s_prev_calc_us;
        s_prev_calc_us = now_us;

        idle0_us = s_idle_us_core0;
        s_idle_us_core0 = 0;

        idle1_us = s_idle_us_core1;
        s_idle_us_core1 = 0;

        if (elapsed_us > 0U) {
            idle0_pct = (idle0_us * 100U) / elapsed_us;
            idle1_pct = (idle1_us * 100U) / elapsed_us;

            if (idle0_pct > 100U) {
                idle0_pct = 100U;
            }
            if (idle1_pct > 100U) {
                idle1_pct = 100U;
            }

            msg.core0_load_pct = (uint8_t)(100U - idle0_pct);
            msg.core1_load_pct = (uint8_t)(100U - idle1_pct);

            if (g_p_cpu_mbox != NULL) {
                xQueueOverwrite(g_p_cpu_mbox, &msg);
            }
        }
    }
}

// -----------------------------------------------------------
// [CPU Core0 FreeRTOSタスク]

/**
 * @brief CPU Core 0: 音声録音・再生およびリアルタイムFFT演算タスク
 */
static void _task_audio(void* p_arg)
{
    int32_t  g12_state;
    int16_t* p_write_target;
    uint32_t play_elapsed_ms;
    uint32_t play_sample_idx;
    uint32_t cur_sec;
    uint16_t copy_len;
    int32_t  i;
    int16_t  sample_chunk[FFT_N];
    float    current_bars[FFT_BAR_COUNT];

    (void)p_arg;

    while (1)
    {
        if ((g_p_record_buffer == NULL) || g_is_sleep_req) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        g12_state = digitalRead(PIN_BTN_RECORD);

        if (g12_state == LOW) {
            /* G12ボタン押下中: 録音 */
            if (!g_is_recording) {
                g_is_recording = true;
                g_recorded_samples = 0;
                M5.Speaker.end();
                M5.Mic.begin();
            }

            if ((g_recorded_samples + AUDIO_CHUNK_SAMPLES) <= RECORD_TOTAL_SAMPLES) {
                p_write_target = ((int16_t*)g_p_record_buffer) + g_recorded_samples;
                M5.Mic.record(p_write_target, AUDIO_CHUNK_SAMPLES, RECORD_SAMPLE_RATE);
                while (M5.Mic.isRecording() && !g_is_sleep_req) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                g_recorded_samples += AUDIO_CHUNK_SAMPLES;
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } else {
            /* G12ボタン非押下 */
            if (g_is_recording) {
                g_is_recording = false;
                M5.Mic.end();

                if ((g_recorded_samples > 0) && !g_is_sleep_req) {
                    g_total_play_sec = (g_recorded_samples + RECORD_SAMPLE_RATE - 1) / RECORD_SAMPLE_RATE;
                    g_play_cur_sec = 0;
                    g_play_start_ms = millis();
                    g_is_playing = true;

                    M5.Speaker.begin();
                    M5.Speaker.setVolume(200);

                    M5.Speaker.playRaw(
                        (const int16_t*)g_p_record_buffer,
                        g_recorded_samples,
                        RECORD_SAMPLE_RATE,
                        false
                    );

                    /* 再生中はCore 0側でFFT演算を実行 */
                    while (M5.Speaker.isPlaying() && !g_is_sleep_req)
                    {
                        play_elapsed_ms = millis() - g_play_start_ms;
                        cur_sec = play_elapsed_ms / 1000;
                        if (cur_sec > g_total_play_sec) {
                            cur_sec = g_total_play_sec;
                        }
                        g_play_cur_sec = cur_sec;

                        play_sample_idx = (play_elapsed_ms * RECORD_SAMPLE_RATE) / 1000;
                        if (play_sample_idx < g_recorded_samples) {
                            copy_len = FFT_N;
                            if ((play_sample_idx + copy_len) > g_recorded_samples) {
                                copy_len = (uint16_t)(g_recorded_samples - play_sample_idx);
                            }
                            memcpy(sample_chunk, ((const int16_t*)g_p_record_buffer) + play_sample_idx, copy_len * sizeof(int16_t));
                            if (copy_len < FFT_N) {
                                memset(sample_chunk + copy_len, 0, (FFT_N - copy_len) * sizeof(int16_t));
                            }
                        } else {
                            memset(sample_chunk, 0, sizeof(sample_chunk));
                        }

                        _calculate_fft_bars(sample_chunk, FFT_N, current_bars);

                        for (i = 0; i < FFT_BAR_COUNT; i++) {
                            if (current_bars[i] >= g_fft_bars[i]) {
                                g_fft_bars[i] = current_bars[i];
                            } else {
                                g_fft_bars[i] = g_fft_bars[i] * 0.78f;
                            }
                        }

                        vTaskDelay(pdMS_TO_TICKS(25));
                    }

                    M5.Speaker.stop();
                    M5.Speaker.end();
                    g_is_playing = false;

                    memset(g_p_record_buffer, 0, RECORD_BUFFER_SIZE);
                    g_recorded_samples = 0;
                    g_total_play_sec = 0;
                    g_play_cur_sec = 0;
                }
            } else {
                /* 待機中はFFTバーを減衰 */
                for (i = 0; i < FFT_BAR_COUNT; i++) {
                    if (g_fft_bars[i] > 0.01f) {
                        g_fft_bars[i] = g_fft_bars[i] * 0.75f;
                    } else {
                        g_fft_bars[i] = 0.0f;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
}

// -----------------------------------------------------------
// [CPU Core1 FreeRTOSタスク]

/**
 * @brief CPU Core 1: センサ取得・非同期DMA UI描画・電源管理タスク
 */
static void _task_ui_motion(void* p_arg)
{
    float           ax;
    float           ay;
    float           az;
    float           gx;
    float           gy;
    float           gz;
    float           accel_norm;
    float           accel_dyn;
    float           dt;
    int32_t         g11_state;
    uint32_t        now_ms;
    uint32_t        rec_sec;
    cpu_load_msg_t  cpu_msg;
    int32_t         disp_w;
    int32_t         disp_h;
    bool            is_need_draw;
    static int32_t  s_battery_pct = 100;
    static uint32_t s_last_bat_ms = 0;
    static uint8_t  s_core0_load = 0;
    static uint8_t  s_core1_load = 0;
    static uint32_t s_last_update_ms = 0;
    static uint32_t s_last_step_ms = 0;
    static uint8_t  s_frame_div = 0;
    static bool     s_is_peak_detected = false;
    static bool     s_is_g11_released = false;

    (void)p_arg;
    s_battery_pct = M5.Power.getBatteryLevel();

    while (1)
    {
        /* FreeRTOSメールボックスからCPU負荷率を非破壊受信 (xQueuePeek) */
        if (g_p_cpu_mbox != NULL) {
            if (xQueuePeek(g_p_cpu_mbox, &cpu_msg, 0) == pdPASS) {
                s_core0_load = cpu_msg.core0_load_pct;
                s_core1_load = cpu_msg.core1_load_pct;
            }
        }

        now_ms = millis();
        dt = (float)(now_ms - s_last_update_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 0.03f;
        }
        s_last_update_ms = now_ms;

        /* バッテリー残量のI2C取得およびM5.updateは1秒周期に間引いてI2C負荷を削減 */
        if ((now_ms - s_last_bat_ms) >= 1000U) {
            M5.update();
            s_battery_pct = M5.Power.getBatteryLevel();
            s_last_bat_ms = now_ms;
        }

        /* BMI270から加速度とジャイロを取得 (33Hz) */
        M5.Imu.getAccel(&ax, &ay, &az);
        M5.Imu.getGyro(&gx, &gy, &gz);

        /* 姿勢に応じた画面回転 */
        _update_rotation(gx, gy, gz, ax, ay);

        /* 合成加速度 (G) の算出 */
        accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));

        /* 歩数判定 */
        if (accel_norm > 1.25f) {
            if (!s_is_peak_detected && ((now_ms - s_last_step_ms) > 300)) {
                g_step_count++;
                s_last_step_ms = now_ms;
                s_is_peak_detected = true;
            }
        } else if (accel_norm < 1.05f) {
            s_is_peak_detected = false;
        }

        /* 速度積算 */
        accel_dyn = fabsf(accel_norm - 1.0f) * 9.80665f;
        if (accel_dyn < 0.25f) {
            accel_dyn = 0.0f;
        }
        g_velocity = (g_velocity * 0.96f) + (accel_dyn * dt);

        /* G11ボタンの監視 */
        g11_state = digitalRead(PIN_BTN_SLEEP);
        if (g11_state == HIGH) {
            s_is_g11_released = true;
        }

        /* DeepSleep判定 */
#if 0
        if ((s_is_g11_released && (g11_state == LOW)) \
            || ((millis() - g_wake_start_ms) >= DEEP_SLEEP_TRG_TIME_MS)
        ) {
            _enter_deep_sleep();
        }
#else
        if (s_is_g11_released && (g11_state == LOW)) {
            _enter_deep_sleep();
        }
#endif

        /* 録音・再生中(FFT表示中)は毎ループ(約30FPS)、通常待機時は2回に1回(約15FPS)描画 */
        s_frame_div++;
        is_need_draw = (g_is_recording || g_is_playing || ((s_frame_div & 1U) == 0U));

        if (is_need_draw) {
            disp_w = M5.Display.width();
            disp_h = M5.Display.height();

            g_canvas.fillScreen(TFT_BLACK);
            g_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
            g_canvas.setFont(&fonts::Font2);
            g_canvas.setTextSize(1);

            if (disp_w < disp_h) {
                /* 縦向き画面 (135 x 240) */
                g_canvas.setCursor(2, 0);
                g_canvas.printf("BAT:%d%% STP:%u\n", (int)s_battery_pct, (unsigned int)g_step_count);
                g_canvas.printf("CPU Core 0: %u%%\n", (unsigned int)s_core0_load);
                g_canvas.printf("CPU Core 1: %u%%\n", (unsigned int)s_core1_load);
                g_canvas.printf("X:%+0.2f Y:%+0.2f\n", ax, ay);
                g_canvas.printf("Z:%+0.2f G\n", az);
                g_canvas.printf("SPD:%0.2f m/s\n", g_velocity);

                g_canvas.setCursor(2, 102);
                if (g_is_recording) {
                    rec_sec = g_recorded_samples / RECORD_SAMPLE_RATE;
                    g_canvas.printf("REC: %u/30s", (unsigned int)rec_sec);
                } else if (g_is_playing) {
                    g_canvas.printf("PLAY: %u/%us", (unsigned int)g_play_cur_sec, (unsigned int)g_total_play_sec);
                }

                _draw_fft_bars(4, disp_h - 112, disp_w - 8, 108, g_fft_bars);

            } else {
                /* 横向き画面 (240 x 135) */
                g_canvas.setCursor(2, 0);
                g_canvas.printf("BAT:%d%% STP:%u\n", (int)s_battery_pct, (unsigned int)g_step_count);
                g_canvas.printf("CPU Core 0: %u%%\n", (unsigned int)s_core0_load);
                g_canvas.printf("CPU Core 1: %u%%\n", (unsigned int)s_core1_load);
                g_canvas.printf("SPD:%0.2f m/s\n", g_velocity);

                g_canvas.setCursor(138, 0);
                g_canvas.printf("X:%+0.2f", ax);
                g_canvas.setCursor(138, 16);
                g_canvas.printf("Y:%+0.2f", ay);
                g_canvas.setCursor(138, 32);
                g_canvas.printf("Z:%+0.2f", az);

                g_canvas.setCursor(2, 64);
                if (g_is_recording) {
                    rec_sec = g_recorded_samples / RECORD_SAMPLE_RATE;
                    g_canvas.printf("REC: %u/30s", (unsigned int)rec_sec);
                } else if (g_is_playing) {
                    g_canvas.printf("PLAY: %u/%us", (unsigned int)g_play_cur_sec, (unsigned int)g_total_play_sec);
                }

                _draw_fft_bars(4, disp_h - 52, disp_w - 8, 48, g_fft_bars);
            }

            /*
             * 非同期SPI-DMA転送とvTaskDelayのオーバーラップ:
             * startWrite()でトランザクションを開いた状態でpushSprite()を呼ぶと、
             * GDMAに転送開始を指示した直後にブロッキングせず即座にリターンする。
             * 続くvTaskDelay(30)でCore 1がスリープしている裏で13.5msのDMA転送が完了する。
             */
            M5.Display.startWrite();
            g_canvas.pushSprite(0, 0);
            vTaskDelay(pdMS_TO_TICKS(30));
            M5.Display.endWrite();
        } else {
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
}

// -----------------------------------------------------------
// [Main]

void setup(void)
{
    m5::M5Unified::config_t cfg;
    mic_cfg_t               mic_cfg;
    uint32_t                wait_start_ms;

    cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_mic = true;
    cfg.internal_spk = true;
    M5.begin(cfg);

    mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = RECORD_SAMPLE_RATE;
    mic_cfg.stereo = false;
    M5.Mic.config(mic_cfg);

    /* DeepSleep復帰後にRTC GPIO設定を解除して通常GPIOへ戻す */
    rtc_gpio_deinit((gpio_num_t)PIN_BTN_SLEEP);
    pinMode(PIN_BTN_SLEEP, INPUT_PULLUP);
    pinMode(PIN_BTN_RECORD, INPUT_PULLUP);

#ifdef M5PM1_L3B_USE
    _setup_pm1_l3b();
#endif

    /* 録音用バッファ (最大30秒) をPSRAMから確保 */
    g_p_record_buffer = (uint8_t*)ps_malloc(RECORD_BUFFER_SIZE);
    if (g_p_record_buffer != NULL) {
        memset(g_p_record_buffer, 0, RECORD_BUFFER_SIZE);
    }

    /* 描画スプライト初期化 (MALLOC_CAP_DMA内部SRAMに配置しゼロコピーGDMAを有効化) */
    M5.Display.setRotation(0);
    g_canvas.setPsram(false);
    g_canvas.createSprite(M5.Display.width(), M5.Display.height());

    /* FreeRTOSメールボックス生成 (要素数 1) */
    g_p_cpu_mbox = xQueueCreate(1, sizeof(cpu_load_msg_t));

    /* 各コアのIdle Hook登録 */
    esp_register_freertos_idle_hook_for_cpu(_idle_hook_core0, 0);
    esp_register_freertos_idle_hook_for_cpu(_idle_hook_core1, 1);

    /* 起床直後: G11が離されるまで待機 (最大2秒) */
    wait_start_ms = millis();
    while ((digitalRead(PIN_BTN_SLEEP) == LOW) && ((millis() - wait_start_ms) < 2000))
    {
        delay(10);
    }

    g_wake_start_ms = millis();

    /* Core 0: 音声処理＋FFTタスク生成 (優先度 2) */
    xTaskCreatePinnedToCore(
        _task_audio,
        "TaskAudio",
        8192,
        NULL,
        2,
        NULL,
        0
    );

    /* Core 1: UI・モーションセンサタスク生成 (優先度 1) */
    xTaskCreatePinnedToCore(
        _task_ui_motion,
        "TaskUIMotion",
        8192,
        NULL,
        1,
        NULL,
        1
    );

    /* Core 0: CPU負荷率計算タスク生成 (優先度 3) */
    xTaskCreatePinnedToCore(
        _task_cpu_load,
        "TaskCpuLoad",
        2048,
        NULL,
        3,
        NULL,
        0
    );
}

void loop(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
// -----------------------------------------------------------