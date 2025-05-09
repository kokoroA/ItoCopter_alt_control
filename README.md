# PicoCopter with VL53L1X Auto Altitude Control

このプロジェクトは、[itolab2022/ItoCopter](https://github.com/itolab2022/ItoCopter) をフォークして拡張したものです。  
Raspberry Pi Picoを用いたマルチコプター制御に、**VL53L1X ToFセンサ**による高度検出機能と、**カルマンフィルタ＋PID制御**による自動高度制御機能を追加しました。

## 特徴

- **VL53L1Xセンサ**を用いた高精度な地上からの距離測定
- **カルマンフィルタ**による高度,高さ方向の速度推定
- **PID制御**による目標高度への追従
- 当初の手動操績に加え、自動での高度維持機能を実装
- Raspberry Pi Pico上で動作

## 構成ファイル

| ファイル名         | 内容                                    |
|:-------------------|:----------------------------------------|
| `control.cpp`       | メインの制御ロジック、高度制御PIDを実装 |
| `ekf.cpp`           | 高度,高さ方向の速度推定用のカルマンフィルタの実装      |
| `pico_copter.cpp`   | メインループ、初期化処理               |
| `sensor.cpp`        | VL53L1XおよびIMUセンサのデータ取得     |
| `radio.cpp`         | RC信号受信（SBUS通信）                  |
| `pwm.cpp`           | モータ制御用PWM出力                  |
| `lsm9ds1_reg.c`     | IMU（LSM9DS1）センサのドライバ         |

## 追加された機能

- **VL53L1Xによる高度計測**  
  `sensor.cpp`内の`initialize_Altitude`、`get_Altitude`関数を使用して、I2C経由で高度を取得しています。

- **カルマンフィルタによるノイズ除去・推定**  
  `ekf.cpp`にて、センサノイズを抑制するためのカルマンフィルタを実装しています。

- **PID制御による目標高度への制御**  
  `control.cpp`にて、推定された高度,高さ方向の速度に基づき、2重のPIDフィードバック制御を適用しています。

## ビルド環境

- マイコン：Raspberry Pi Pico
- センサ：
  - VL53L1X (ToFセンサ)
  - LSM9DS1 (IMUセンサ)
- IDE/ツール：
  - CMake
  - Pico SDK
- 開発環境：
  - Raspberry Pi OS / Ubuntu / Windows + WSL推奨

## 配線

- **VL53L1X**：
  - I2C接続（SDA/SCLピンに接続）
- **LSM9DS1**：
  - SPI接続
- **RC受信機**：
  - UART接続（SBUS対応）

## 動作イメージ

1. 起動後、自動でVL53L1Xセンサが初期化されます。
2. リモコンのスティック操作なしでも、一定高度を維持する動作が可能です。
3. RC入力によりマニュアル操作も可能です。
