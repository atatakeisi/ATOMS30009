# ATOMS30009 — ATOM S3 + SCS0009 四足歩行ロボット WiFiコントローラー

M5Stack ATOM S3 と SCS0009 サーボ8軸による四足歩行ロボット。
WiFi アクセスポイント + ブラウザ(HTTP)でリアルタイム制御する。
2リンク逆運動学(IK)で各脚を制御し、内蔵IMU(MPU6886)で表裏判定・姿勢制御を行う。

> このドキュメントは **現在の `src/main.cpp` の構成** を一から記述したもの。
> 過去の別設計(WebSocket版・サーボグループ版など)とは無関係。

---

## ⚠️ 最重要の注意事項：サーボTXは GPIO5。GPIO38 は絶対に使わない

サーボバスの TX は **GPIO5(`TX_PIN = 5`)** に配線する(ハードウェア改造済み)。
**GPIO38 をサーボTXに使ってはいけない。** 過去にG38へTX設定して動かなくなった失敗がある。理由は2つ:

1. **GPIO38 は内蔵IMU(MPU6886)の I2C SDA**(SCL=G39)。ここをサーボTXに奪うと
   **IMUが使えなくなる**(表裏判定・IMU水平制御が死ぬ)。
2. **GPIO38 は FSPIWP を兼ねる**。`WiFi.softAP()` が内部でFSPIを初期化する際に
   G38を奪うため、その後 `Serial1.begin(TX=38)` を呼んでも **UART TXとして機能しない**。

→ サーボTXを **GPIO5** に移したことで、IMUとサーボを同時に使えるようになった。
G5にはこれらの競合がないため、`Serial1.begin()` と `WiFi.softAP()` の呼び出し順は問わない。

```cpp
#define TX_PIN 5
Serial1.begin(1000000, SERIAL_8N1, -1, TX_PIN);  // RXは未接続(WRITEのみ・応答不要)
// 内蔵IMU: MPU6886  SDA=G38 / SCL=G39
```

---

## ハードウェア構成

| 部品 | 仕様 |
|------|------|
| マイコン | M5Stack ATOM S3(ESP32-S3, 240MHz, 128x128 LCD) |
| サーボ | SCS0009 × 8(ID 0〜7、半二重UART 1Mbps、TX=GPIO5) |
| IMU | 内蔵 MPU6886(SDA=G38 / SCL=G39) |
| 電源/通信 | WiFi AP(SSID: `robot0009` / PW: `password`) |
| IPアドレス | `192.168.55.27` |

---

## サーボマップ（実機計測で確定済み）

各サーボの ID・取り付け位置・「水平になる生値(raw)」・「raw を増やしたときの動く向き」を
実機で計測して確定した。`offset = 水平RAW - 511`(511が機械中央)。

| ID | 脚 | 関節 | 水平RAW | +raw方向 | offset |
|----|-----|------|--------|---------|--------|
| 0 | 後左 RL | 肩 shoulder | 476 | 下がる | **-35** |
| 1 | 後右 RR | 肩 shoulder | 511 | 上がる | **0** |
| 2 | 後左 RL | 膝 knee | 524 | 下がる | **+13** |
| 3 | 後右 RR | 膝 knee | 504 | 上がる | **-7** |
| 4 | 前右 FR | 肩 shoulder | 564 | 下がる | **+53** |
| 5 | 前左 FL | 肩 shoulder | 467 | 上がる | **-44** |
| 6 | 前右 FR | 膝 knee | 526 | 下がる | **+15** |
| 7 | 前左 FL | 膝 knee | 545 | 上がる | **+34** |

これらの offset は `main.cpp` の初期値にも入っているが、Webのキャリブレーションで
調整した値が NVS(Preferences) に保存され、起動時はそちらが優先される。

### ATOMS3 の向きと脚の対応（画面を上に向けた状態 = 正立）

```
                  後ろ REAR（画面の上側）
   画面左 = 機体右                       画面右 = 機体左
   ┌─ RR 後右 ─┐                        ┌─ RL 後左 ─┐
   │ 肩ID1 膝ID3│                        │ 肩ID0 膝ID2│
   └───────────┘   ┌───────────────┐   └───────────┘
                    │   ATOMS3 LCD   │
                    │ (画面上=正立)  │
                    └───────────────┘
   ┌─ FR 前右 ─┐                        ┌─ FL 前左 ─┐
   │ 肩ID4 膝ID6│                        │ 肩ID5 膝ID7│
   └───────────┘                        └───────────┘
                  前 FRONT（画面の下側）
```

- 画面の **上=機体後ろ / 下=機体前**
- 画面の **左=機体右(ID 1,3,4,6) / 右=機体左(ID 0,2,5,7)**

---

## 逆運動学(IK)と符号規則

`legIK(x, z)` が脚先目標 (x=前後, z=下向き深さ) から2つの角度を返す:

- **th1**(肩): `0` で腿が水平、**+で脚が下がる**
- **th2**(膝): `0` で脚がまっすぐ、**+で膝が曲がって足が下がる**

角度0は実機キャリブレーションの「水平」姿勢に一致する(offsetで校正済み)。
到達不能時に `acos` が NaN にならないよう引数を ±1 にクランプしている。

### 取り付け符号（実機計測ベース）

サーボへ渡す角度の符号は「**+raw方向が下がる → +th**」「**+raw方向が上がる → -th**」:

| 脚 | 肩 | 膝 | 符号 |
|----|----|----|------|
| 後左 RL | ID0 | ID2 | **(+th1, +th2)** |
| 前右 FR | ID4 | ID6 | **(+th1, +th2)** |
| 後右 RR | ID1 | ID3 | **(-th1, -th2)** |
| 前左 FL | ID5 | ID7 | **(-th1, -th2)** |

→ 対角 **RL+FR が (+,+)**、対角 **RR+FL が (-,-)**。
`pos==1`(裏返し)時は本体が上下逆になるため全符号を反転する。

---

## 表裏判定（pos）と IMU 軸

`get_theta()` が加速度から表裏を判定する。**ATOMS3/MPU6886 は Z軸の向きが
原作(ATOM Matrix)と逆**で、画面上向き(正立)で `accZ≈+1g`、裏返しで `accZ≈-1g`:

```cpp
if(pos == 1 && accZ >  0.96) pos = 0;  // 画面上向き = 正立
if(pos == 0 && accZ < -0.96) pos = 1;  // 裏返し
```

- `theta_Y`(ロール)と gyro は実機計測で符号反転済み。
- `theta_X`(ピッチ)の符号は **未検証**。IMUモードで前後に傾けて、水平に戻る向きに
  脚が動けば正。**傾きを増幅する(悪化する)なら `theta_X` の符号反転が必要**(TODO)。

---

## 安全機構（速度制御 と ペーシング）

SCS0009 は **速度バイト = 0 で「最高速＝瞬間移動」** になり、指を挟む危険がある。
本ファームでは2段階で安全化している:

1. **速度制限 `servoSpeed`**: 位置指令パケットの速度バイトに常に載せる。
   小さいほどゆっくり。Webの `speed` で調整・NVS保存。起動時に安全範囲へクランプ。
2. **ペーシング**: `servo_write()` の末尾に `delayMicroseconds(1500)`。
   歩行ループが待ち時間ゼロでコマンドを連射し、速度制御付きサーボが
   毎回モーション再スタートして暴走するのを防ぐ(全モード約80Hzに統一)。

> **注意**: Jump / Roll は本来フルスピードの跳躍・反転ダイナミクス前提。
> 速度制限下では跳べず極端姿勢をなぞるだけになる。**床に置き、速度を上げて慎重に**。

### SCS0009 位置指令パケット

```
FF FF  ID  09  03  2A  posH posL  timeH timeL  spdH spdL  CS
 ヘッダ ID  LEN WRITE  addr42 ─位置─  ─時間(0)─  ─速度─    チェックサム
```

レジスタ42から 位置(42/43)・時間(44/45)・速度(46/47) を一括書き込み。
本ファームは時間=0、速度=`servoSpeed`(ビッグエンディアン、posと同じ並び)。

---

## Web UI（すべてPOST）

> **重要**: 動作系のリンクを `<a href>`(GET) にすると、Safari等の **リンク先読み(prefetch)**
> でページを開いた瞬間に勝手に動作が発火して暴れる。**全てPOSTフォーム + POST専用ルート**
> にしてある。ページ表示 `/` だけが GET。先読みは GET しか行わないので誤発火しない。

`192.168.55.27` を開くと、上から:

- **speed** [-][+] … サーボ速度(安全のため低めから)
- **RESET** … 制御パラメータを初期値へ(period/upHeight/stride/speed/Kp/Kd/wakeUpAngle)。**offsetは消さない**
- **CALIB / STAND** … 全サーボ水平保持 / IKによる立ち姿勢(緑=選択中)
- **x / height** … 立ち姿勢(STAND)の脚先 前後位置・高さ
- **歩行ボタン** … Step / Stretch / Advance / Back / L / R / IMU / Jump / Roll
- **period / upHeight / stride** … 歩行パラメータ
- **servo select / offset** … 個別サーボの較正(◀ID / ID▶、-10/-1/+1/+10、reset)

動作ボタンは押すと該当モードへ。同じボタン再押しで停止(Standby)。

---

## キャリブレーション手順

1. 起動直後は **CALIB**(全サーボ水平保持)。
2. `servo select` で ID を選び、`-10/-1/+1/+10` でそのサーボだけ動かす。
   どの脚のどの関節が、どちらに動くかを1個ずつ確認できる。
3. すべての脚が水平になるよう offset を調整(値は即 NVS 保存)。
4. **STAND** を押して正しく立つか確認。
5. LCD には CALIB中は `ID / off / raw`、他モードでは モード名 + 傾き + 水準器バブルを表示。

---

## パラメータの永続化（NVS / Preferences）

起動時に NVS から読み込む: `offset0..7, period, x, height, upHeight, stride,
servoSpeed, Kp, Kd, wakeUpAngle`。読み込み後、暴走防止のため安全範囲へクランプ
(特に `period` は40未満にならないよう補正)。

- **RESET ボタン**: 制御パラメータを初期値に戻す。**offset(較正値)は保持**。
- 残留した極端値(例: 過去テストの `period=5`)で歩行が暴走するのを防ぐ。

---

## ビルド / 書き込み

```bash
pio run                 # ビルド
pio run -t upload       # 書き込み(ATOMS3 をUSB接続)
pio device monitor      # シリアル(115200)。acc/theta/kal/pos をデバッグ出力
```

`platformio.ini`:

```ini
[env:m5stack-atoms3]
platform = espressif32
board = m5stack-atoms3
framework = arduino
build_flags = -DARDUINO_USB_CDC_ON_BOOT=1
lib_deps =
  m5stack/M5Unified
  https://github.com/TKJElectronics/KalmanFilter.git
```

---

## ファイル構成

```
ATOMS30009/
├── README.md          ← このファイル
├── platformio.ini     ← ビルド設定
└── src/
    └── main.cpp       ← 本体(WiFi AP + HTTP + IK歩行 + IMU + 較正)
```

---

## 既知のTODO（後日調整）

- **Step のふらつき**: 足の設置が微ズレ。offset / height / upHeight で微調整。
- **IMUモードの `theta_X`(ピッチ)符号**: 実機で傾けて、悪化方向なら反転。
- **Jump / Roll**: フルスピード前提。床で速度を上げて慎重に検証・調整。
- **pos==1(裏返し)時の歩行**: STAND の反転は確認済みだが歩行は未検証。
