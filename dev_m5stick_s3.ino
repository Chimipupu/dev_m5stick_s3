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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// -----------------------------------------------------------
#define PIN_BTN_SLEEP        (11) /* 起床およびスリープ用ボタン (G11) */
#define PIN_BTN_RECORD       (12) /* 録音操作用ボタン (G12) */

/* 音声サンプリング設定 (16kHz, 16bit モノラル: 30秒で 960,000 bytes) */
#define RECORD_SAMPLE_RATE   (16000)
#define RECORD_MAX_SECONDS   (30)
#define RECORD_TOTAL_SAMPLES (RECORD_SAMPLE_RATE * RECORD_MAX_SECONDS)
#define RECORD_BUFFER_SIZE   (RECORD_TOTAL_SAMPLES * sizeof(int16_t))
#define AUDIO_CHUNK_SAMPLES  (512) /* 1チャンクあたり32ms */

uint8_t* g_p_record_buffer = NULL;
uint32_t g_recorded_samples = 0;
uint32_t g_step_count = 0;
float    g_velocity = 0.0f;
uint32_t g_wake_start_ms = 0;
bool     g_is_recording = false;
bool     g_is_playing = false;
M5Canvas g_canvas(&M5.Display);

static void _enter_deep_sleep(void);
static void _update_rotation(float gx, float gy, float gz, float ax, float ay);
static void _task_audio(void* p_arg);
static void _task_ui_motion(void* p_arg);

// -----------------------------------------------------------
// [Static]

/**
 * @brief DeepSleepへ移行
 */
static void _enter_deep_sleep(void)
{
    /* 画面消灯 */
    M5.Display.sleep();
    M5.Display.waitDisplay();

    /* G11ピンがLOW (ボタン押下) になったときに起床するよう設定 */
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BTN_SLEEP, 0);

    /* DeepSleep開始 */
    esp_deep_sleep_start();
}

/**
 * @brief ジャイロおよび加速度センサの値からLCD表示方向を自動回転
 */
static void _update_rotation(float gx, float gy, float gz, float ax, float ay)
{
    int32_t        target_rotation;
    static int32_t s_active_rotation = 0;

    target_rotation = s_active_rotation;

    /* 重力方向および傾きから端末の向き (0:縦, 1:横, 2:縦逆, 3:横逆) を判定 */
    if (fabsf(ax) > fabsf(ay)) {
        if (ax > 0.5f) {
            target_rotation = 1;
        } else if (ax < -0.5f) {
            target_rotation = 3;
        }
    } else {
        if (ay > 0.5f) {
            target_rotation = 0;
        } else if (ay < -0.5f) {
            target_rotation = 2;
        }
    }

    /* 向きが変化した場合のみ再描画用スプライトを再確保 */
    if (target_rotation != s_active_rotation) {
        s_active_rotation = target_rotation;
        M5.Display.setRotation(s_active_rotation);
        g_canvas.deleteSprite();
        g_canvas.createSprite(M5.Display.width(), M5.Display.height());
    }
}

// -----------------------------------------------------------
// [CPU Core0 FreeRTOSタスク]

/**
 * @brief CPU Core 0: 音声録音および再生タスク
 */
static void _task_audio(void* p_arg)
{
    int32_t  g12_state;
    int16_t* p_write_target;

    (void)p_arg;

    while (1)
    {
        if (g_p_record_buffer == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
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

            /* PSRAMバッファに空きがあればDMA録音を実行し完了を待機 */
            if ((g_recorded_samples + AUDIO_CHUNK_SAMPLES) <= RECORD_TOTAL_SAMPLES) {
                p_write_target = ((int16_t*)g_p_record_buffer) + g_recorded_samples;
                M5.Mic.record(p_write_target, AUDIO_CHUNK_SAMPLES, RECORD_SAMPLE_RATE);
                while (M5.Mic.isRecording()) {
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

                if (g_recorded_samples > 0) {
                    g_is_playing = true;
                    M5.Speaker.begin();
                    M5.Speaker.setVolume(200);

                    /* 録音データを同期再生 */
                    M5.Speaker.playRaw(
                        (const int16_t*)g_p_record_buffer,
                        g_recorded_samples,
                        RECORD_SAMPLE_RATE,
                        false
                    );

                    while (M5.Speaker.isPlaying()) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }

                    M5.Speaker.stop();
                    M5.Speaker.end();
                    g_is_playing = false;

                    /* 再生完了後にPSRAM領域をメモリクリア */
                    memset(g_p_record_buffer, 0, RECORD_BUFFER_SIZE);
                    g_recorded_samples = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// -----------------------------------------------------------
// [CPU Core1 FreeRTOSタスク]

/**
 * @brief CPU Core 1: センサ取得・UI描画・電源管理タスク
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
    int32_t         battery_pct;
    int32_t         g11_state;
    uint32_t        now_ms;
    static uint32_t s_last_update_ms = 0;
    static uint32_t s_last_step_ms = 0;
    static bool     s_is_peak_detected = false;
    static bool     s_is_g11_released = false;

    (void)p_arg;

    while (1)
    {
        M5.update();

        now_ms = millis();
        dt = (float)(now_ms - s_last_update_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 0.03f;
        }
        s_last_update_ms = now_ms;

        /* BMI270から加速度とジャイロを取得 */
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

        /* バッテリー残量取得 */
        battery_pct = M5.Power.getBatteryLevel();

        /* LCD描画 (黒背景・白文字・TextSize: 2) */
        g_canvas.fillScreen(TFT_BLACK);
        g_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        g_canvas.setTextSize(2);
        g_canvas.setCursor(2, 2);

        /* フォーマット警告回避のため明示的キャスト */
        g_canvas.printf("BAT:%d%%\n", (int)battery_pct);
        g_canvas.printf("STP:%u\n", (unsigned int)g_step_count);
        g_canvas.printf("ACC(G):\n");
        g_canvas.printf("X:%+0.2f\n", ax);
        g_canvas.printf("Y:%+0.2f\n", ay);
        g_canvas.printf("Z:%+0.2f\n", az);
        g_canvas.printf("SPD:%0.2f\n", g_velocity);

        if (g_is_recording) {
            g_canvas.drawString("[REC]", 2, M5.Display.height() - 18);
        } else if (g_is_playing) {
            g_canvas.drawString("[PLAY]", 2, M5.Display.height() - 18);
        }

        g_canvas.pushSprite(0, 0);

        /* G11ボタンの監視 */
        g11_state = digitalRead(PIN_BTN_SLEEP);
        if (g11_state == HIGH) {
            s_is_g11_released = true;
        }

        /* DeepSleep判定 */
#if 0
        /* 5秒経過またはG11再押下でDeepSleep */
        if ((s_is_g11_released && (g11_state == LOW)) || ((millis() - g_wake_start_ms) >= 5000)) {
            _enter_deep_sleep();
        }
#else
        /* 現在: G11再押下時のみDeepSleepへ移行 */
        if (s_is_g11_released && (g11_state == LOW)) {
            _enter_deep_sleep();
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
// -----------------------------------------------------------
// [Main]

void setup(void)
{
    m5::M5Unified::config_t       cfg;
    decltype(M5.Mic.config())     mic_cfg;
    uint32_t                      wait_start_ms;

    cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_mic = true;
    cfg.internal_spk = true;
    M5.begin(cfg);

    mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = RECORD_SAMPLE_RATE;
    mic_cfg.stereo = false;
    M5.Mic.config(mic_cfg);

    pinMode(PIN_BTN_SLEEP, INPUT_PULLUP);
    pinMode(PIN_BTN_RECORD, INPUT_PULLUP);

#ifdef M5PM1_L3B_USE
    /* PMIC M5PM1のL3B設定 */
    _setup_pm1_l3b();
#endif

    // 録音用バッファ(最大30秒)をPSRAMからmalloc()
    g_p_record_buffer = (uint8_t*)ps_malloc(RECORD_BUFFER_SIZE);
    if (g_p_record_buffer != NULL) {
        memset(g_p_record_buffer, 0, RECORD_BUFFER_SIZE);
    }

    // 描画スプライト初期化
    M5.Display.setRotation(0);
    g_canvas.createSprite(M5.Display.width(), M5.Display.height());

    // 起床直後: G11が押されたままであれば離されるまで待機 (最大1秒)
    wait_start_ms = millis();
    while ((digitalRead(PIN_BTN_SLEEP) == LOW) && ((millis() - wait_start_ms) < 1000))
    {
        delay(10);
    }

    g_wake_start_ms = millis();

    /* Core 0: 音声処理タスク生成 (優先度 2) */
    xTaskCreatePinnedToCore(
        _task_audio,
        "TaskAudio",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    /* Core 1: UI・モーションセンサタスク生成 (優先度 1) */
    xTaskCreatePinnedToCore(
        _task_ui_motion,
        "TaskUIMotion",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

void loop(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
// -----------------------------------------------------------