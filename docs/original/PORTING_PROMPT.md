# 四足ロボ：原作コード（ATOM Matrix）→ ATOMS3 移植指示書

これは Claude Code への作業指示書です。下記仕様に従って、原作者のコード（ATOM Matrix / ArduinoIDE 用）を ATOMS3 / PlatformIO 用に移植してください。

---

## 0. 前提・背景

- ロボット：SCS0009 サーボ8個のデイジーチェーン駆動・四足歩行ロボ
- マイコン：M5Stack **ATOMS3**（原作は ATOM Matrix）
- 原作コードは ArduinoIDE 用。`robot0009_original.ino` としてプロジェクト直下に置いてあるので参照すること（無い場合はユーザーに依頼）
- 現在の main.cpp（独自実装版）は **バックアップを取ってから** 置き換えること（例：`src/main_old.cpp.bak` にリネーム、またはgitブランチを切る）
- ハードウェア改造により、サーボTX信号は **GPIO5** に移設済み（G38から変更）。これにより内蔵IMU（MPU6886, I2C: SDA=G38/SCL=G39）との競合が解消され、IMUとサーボが同時に使える

## 1. ビルド環境

- VSCode + PlatformIO。既存の `platformio.ini` の env はそのまま使う（board = m5stack-atoms3, framework = arduino, M5Unified 使用）
- `lib_deps` に Kalman フィルタライブラリを追加：
  ```
  lib_deps =
      m5stack/M5Unified
      https://github.com/TKJElectronics/KalmanFilter.git
  ```
- 原作が使っていた `WebServer.h`（同期版）と `Preferences.h` は ESP32-S3 でそのまま使える。ESPAsyncWebServer は今回は使わない（原作構成を忠実に再現するため）

## 2. 原作からの移植マッピング

| 原作（ATOM Matrix / M5Atom.h） | 移植先（ATOMS3 / M5Unified） |
|---|---|
| `#include <M5Atom.h>` | `#include <M5Unified.h>` |
| `M5.begin(true,true,true)` | `auto cfg = M5.config(); M5.begin(cfg);` |
| `M5.IMU.Init()` | 不要（M5Unifiedが自動初期化。`M5.Imu.isEnabled()`で確認） |
| `M5.IMU.getAccelData(&x,&y,&z)` | `M5.Imu.getAccel(&x,&y,&z)` |
| `M5.IMU.getGyroData(&x,&y,&z)` | `M5.Imu.getGyro(&x,&y,&z)` |
| `M5.IMU.SetAccelFsr / SetGyroFsr` | 省略可（デフォルトで開始。必要なら後で検討） |
| `M5.dis.*`（5x5 LEDマトリクス） | `M5.Display.*`（128x128 LCD）に置換。詳細は §6 |
| `Serial1.begin(1000000, SERIAL_8N1, -1, 19)` | `Serial1.begin(1000000, SERIAL_8N1, -1, TX_PIN)` で `#define TX_PIN 5` |

それ以外のロジック（カルマンフィルタ、IK計算、各モードの軌道生成、Webサーバー、Preferences、Core0タスク分割）は**原作の構造を変えずに**移植する。

## 3. 【最重要】実機固有の修正：肩サーボの符号反転

実機検証の結果、**膝サーボ（ID2,3,6,7）は原作と同じ向き、肩サーボ（ID0,1,4,5）は全て原作と逆向き**に取り付けられていることが確定している。よって4つのIK関数で `th1`（肩）の符号だけを反転する。`th2`（膝）は原作のまま。

置き換え後の servo_write 呼び出し部分（pos==0 / pos==1 の両分岐とも）：

```cpp
// fRIK（右前脚）
if(pos == 0){
    servo_write(6,  th2);    // 原作のまま
    servo_write(4, -th1);    // 原作 +th1 → -th1 に反転
}else{
    servo_write(6, -th2);
    servo_write(4,  th1);    // 原作 -th1 → +th1 に反転
}

// fLIK（左前脚）
if(pos == 0){
    servo_write(7, -th2);    // 原作のまま
    servo_write(5,  th1);    // 原作 -th1 → +th1 に反転
}else{
    servo_write(7,  th2);
    servo_write(5, -th1);
}

// rRIK（右後脚）
if(pos == 0){
    servo_write(3, -th2);    // 原作のまま
    servo_write(1,  th1);    // 原作 -th1 → +th1 に反転
}else{
    servo_write(3,  th2);
    servo_write(1, -th1);
}

// rLIK（左後脚）
if(pos == 0){
    servo_write(2,  th2);    // 原作のまま
    servo_write(0, -th1);    // 原作 +th1 → -th1 に反転
}else{
    servo_write(2, -th2);
    servo_write(0,  th1);
}
```

`constrain` の範囲指定は原作のまま維持する（th1/th2 に対する角度制限）。

## 4. オフセット

実機は「パルス511＝全脚が水平に伸びた状態」になるよう組み立て済み。よって：

```cpp
//           ID   0  1  2  3  4  5  6  7
int offset[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // 実機は511=水平で組立済み
```

原作者の値（-8,-12,6,22,...）は使わない。微調整が必要になったらここを±数カウントで調整する。

## 5. 安全クランプ（物理限界）

`servo_write()` 内、`scs_moveToPos()` を呼ぶ直前にパルス値の上下限クランプを入れる：

```cpp
// 物理限界（仮値。実機で各サーボの可動域を測って絞り込むこと → TODO）
const int POS_MIN[8] = {60, 60, 60, 60, 60, 60, 60, 60};
const int POS_MAX[8] = {960, 960, 960, 960, 960, 960, 960, 960};

void servo_write(int ch, float ang){
  int sig = 511 + offset[ch] + int((512.0 / 150.0) * ang);
  sig = constrain(sig, POS_MIN[ch], POS_MAX[ch]);
  scs_moveToPos(ch, sig);
}
```

注意：Jump / Roll モードは大きな角度（±85°〜107° ≒ ±290〜365カウント）を使うため、限界値を狭くしすぎるとこれらのモードが破綻する。初期値は広め（60〜960）にしておき、実測後に絞る。

## 6. LED表示 → LCD表示の置き換え

原作の browser タスク内 5x5 LEDマトリクス表示（kalAngleX/Y の傾きバー表示）は、M5.Display への簡易表示に置き換える：

- 上段：モード名（文字列）
- 中段：kalAngleX / kalAngleY を数値表示（小数1桁）
- 余裕があれば傾きを水準器風の点やバーで描画（凝らなくてよい）
- 更新間隔は原作と同じ delay(50)〜100ms 程度。ちらつき防止に `M5.Display.fillRect` などで部分更新するか、`startWrite/endWrite` を使う

## 7. IMU軸キャリブレーション（TODO・ビルド後にユーザーと実施）

ATOM Matrix と ATOMS3 では MPU6886 の実装向きが異なる可能性が高い。原作の以下の式・閾値はそのまま移植した上で、**実機テストで符号・軸を確認するためのデバッグ出力を仕込む**こと：

```cpp
theta_X =  atan(accY / -accZ) * 57.29578f;
theta_Y = -atan(-accX / -accZ) * 57.29578f;
// 反転検知（原作）: 正立で pos=0、裏返しで pos=1
if(pos == 1 && accZ < -0.96) pos = 0;
if(pos == 0 && accZ > 0.96) pos = 1;
```

確認手順（シリアルモニタに accX/Y/Z を出力して実施）：
1. ロボットを水平に正立させ accZ の符号を記録（原作想定：約 -1.0）
2. 裏返して accZ を記録（原作想定：約 +1.0）
3. 符号が逆なら pos 判定の不等号と theta 式の符号を修正
4. 前後左右に傾けて theta_X / theta_Y の符号が「前傾＝想定方向」になるよう調整

この確認が済むまで IMU モードと Jump は実行しないようユーザーに伝える。

## 8. その他は原作準拠

- WiFi AP：SSID `robot0009` / pass `password` / IP `192.168.55.27`（原作のまま）
- Web UI：原作の HTML（IMU/Advance/Jump/L/Step/R/Stretch/Back/Roll ボタン＋パラメータ±調整）をそのまま
- Preferences によるパラメータ保存（wakeUpAngle, period, x, height, upHeight, stride, Kp, Kd）も原作のまま
- Core0 に browser タスク、loop() でモード実行という構造も原作のまま

## 9. 段階テスト計画（ユーザーと一緒に実施）

1. ビルド・書き込み → AP接続・Web UI 表示確認
2. 起動直後の初期姿勢 `fRIK(x,height)` 等（x=60, height=40）で四脚が左右対称のM字で立つか
3. Stretch（屈伸）→ 全脚が同位相で滑らかに上下するか
4. Step（足踏み）→ 対角2脚ずつ交互に、肩・膝が連動して足踏みするか
5. Advance / Back / L / R
6. §7 のIMUキャリブレーション → IMUモード（姿勢維持）
7. 最後に Jump / Roll（負荷大。POS_MIN/MAX 実測後が望ましい）

各段階で異常があれば止めて報告すること。脚が暴れる場合は真っ先に §3 の符号と §7 の軸を疑う。
