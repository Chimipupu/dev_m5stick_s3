# M5StickS3 評価F/W開発

## 実装ベースの叩き台

[機能]
- M5Stick S3は普段はDeepSleep
- 正面ボタン(G11ピン)のボタンで起床、もう1回押したらまたDeepSleep
- 側面ボタン(G12ピン)が押されてる間に録音(最大30秒でPSRAMに保存)、ボタンが離されると録音を再生
- LCDは黒背景の白文字、M5Stick S3の向きに応じてBMI270のジャイロを使って回転
- M5Stick S3のバッテリの残量をパーセントをLCD表示
- BMI270から歩数、3軸の加速度を取得してLCD表示

[FreeRTOS]
- CPU Core 0
    - 音声録音/再生タスク
- CPU Core 1
    - LCDとIMU処理の処理タスク