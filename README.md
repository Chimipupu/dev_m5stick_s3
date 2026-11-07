# M5StickS3 評価F/W開発

## 実装ベースの叩き台

[評価・研究結果]
- CPU Core 0で、録音音声をFFTしてもFPUで処理するのでCPU負荷率はほぼなくMAXで7%
- CPU Core 1は、LCDの描画とIMUのデータの取得、計算でCPU負荷率は18%、FFTの表示中は30～35%

[機能]
- M5Stick S3は普段はDeepSleep
- 正面ボタン(G11ピン)のボタンで起床、もう1回押したらまたDeepSleep
- 側面ボタン(G12ピン)が押されてる間に録音(最大30秒でPSRAMに保存)、ボタンが離されると録音を再生
- 録音した音声はCPU Core 0で、FFTして再生時にFFTスペクトラムバーグラフをLCDに描画
- LCDは黒背景の白文字、M5Stick S3の向きに応じてBMI270のジャイロを使って回転
- M5Stick S3のバッテリの残量をLCD表示
- BMI270の歩数、3軸の加速度をLCD表示
- CPU Core 0、Core 1それぞれのCPU負荷率をLCD表示

[FreeRTOS]
- CPU Core 0
    - 音声録音/再生タスク
    - CPU Core 0のCPU負荷率計算タスク
- CPU Core 1
    - LCDとIMU処理の処理タスク
    - CPU Core 0のCPU負荷率計算タスク
- CPU間通信
    - FreeRTOSのメールボックスを使って実施