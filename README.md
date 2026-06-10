# ATOMS3_SCS0009 — M5Stack ATOM S3 四足歩行ロボット WiFiコントローラー

M5Stack ATOM S3 と SCS0009 サーボ 8軸を使用した四足歩行ロボットの  
iPhone（ブラウザ）WebSocket リアルタイム制御システム。

---

## ハードウェア構成

| 部品 | 仕様 |
|------|------|
| マイコン | M5Stack ATOM S3（ESP32-S3FN8, 240MHz） |
| サーボ | SCS0009 × 8（ID: 0〜7、半二重UART 1Mbps） |
| 基板 | カスタムPCB（GPIO38固定 = サーボTX専用） |
| 通信 | WiFi AP（SSID: robot0009 / PW: password） |
| IPアドレス | 192.168.55.27 |

### サーボ接続

```
ATOM S3 GPIO38 (TX) ──── サーボバス（半二重）
                    RX = 未接続（TX専用、SCS0009はWRITE時に応答なし）
```

SCS0009 はデフォルト Status Return Level = 1（READ命令にのみ応答）。  
WRITE（位置指令）には応答パケットを返さないため RX 不要。

### サーボグループ定義（実機レイアウトに合わせて調整）

```
対角A (GA): IDs {0,1,6,7} — Front-Left + Back-Right
対角B (GB): IDs {2,3,4,5} — Front-Right + Back-Left
左側  (GL): IDs {0,1,4,5} — Left side
右側  (GR): IDs {2,3,6,7} — Right side
```

---

## 開発経緯

### Phase 1｜SCServo ライブラリ使用（動作せず）

プロジェクト開始時は公式 SCServo ライブラリを使用していたが、  
全く動作しないという状態が続いた。

**原因1: GPIO38 と WiFi の競合**

ESP32-S3 の GPIO38 は `FSPIWP`（SPI フラッシュ ライトプロテクト）を兼ねる。  
`WiFi.softAP()` が内部で FSPI を初期化する際に GPIO38 を奪い取るため、  
その後に `Serial1.begin()` を呼んでも UART TX として機能しなかった。

```
WiFi.softAP()  →  ESP32内部でFSPI初期化  →  GPIO38をFSPIWPに割り当て
Serial1.begin(TX=38)  →  GPIO38はすでにFSPIが使用中  →  TX不能
```

**解決策**: `Serial1.begin()` を `WiFi.softAP()` の **後に** 呼ぶ。

```cpp
WiFi.softAP(ssid, pass);
delay(200);                            // WiFi先に完了させる
Serial1.begin(1000000, SERIAL_8N1, -1, TX_PIN);  // UARTは後から
```

**原因2: SCServo ライブラリの ACK タイムアウト**

SCServo ライブラリはデフォルトで各サーボからの応答を 100ms 待つ（`IOTimeOut = 100`）。  
8軸 × 100ms = 800ms/サイクル のボトルネック。さらに SCS0009 が  
WRITE コマンドに応答を返さないため、毎回タイムアウト待ちが発生していた。

**解決策**: ライブラリを完全に除去し、生 UART パケットで直接制御。

---

### Phase 2｜servo_test.cpp（単体動作確認 → 完全成功）

ライブラリを除去し、SCS0009 プロトコルを生パケットで実装した  
スタンドアロンテストコード（`src/servo_test.cpp`）を作成。

**確認できた重要事項:**

| 項目 | 内容 |
|------|------|
| `time = 0` は動作しない | 最低 200ms 以上必要（SCS0009の仕様） |
| `Serial1.flush()` 必須 | 各パケット送信後に flush しないと欠落する |
| ボーレート 1Mbps | SCS0009 標準、これ以外では動作しない |
| ブロードキャスト ID 0xFE | 全サーボ同時制御に使用 |
| 動作確認速度 | 30ms サイクルまで正常動作を確認 |

**SCS0009 パケット構造（位置指令）:**

```
FF FF  ID  09  03  2A  posH posL  timeH timeL  spdH spdL  CS
  ↑    ↑   ↑   ↑   ↑   ↑          ↑           ↑          ↑
Header ID LEN  WRITE  アドレス42  移動時間[ms]  速度(0=auto)  チェックサム
```

このコードは `[env:servo_test]` でビルド・書き込みできる。  
WiFi 制御の問題が生じた際のハードウェア確認用として保持。

---

### Phase 3｜WiFi + WebSocket 制御（完成版 `src/main.cpp`）

servo_test.cpp の知見をそのまま移植し、WiFi AP + WebSocket を追加。

**設計上の重要決定:**

- **ESPAsyncWebServer**: Core0 で動作（WebSocket受信・HTTP配信）
- **loop()**: Core1 で動作（サーボ制御ループ）
- **`volatile uint8_t mode`**: Core間の状態共有フラグ
- **`waitM(m, ms)`**: delay の代わりに 10ms ごとに mode を確認 → ボタン即応答
- **モード整数方式**: 複数ボタンの排他制御を1変数で管理

```cpp
// 10ms ごとに mode を確認し、変わったら即抜ける
static bool waitM(uint8_t m, int ms) {
  unsigned long end = millis() + ms;
  while ((long)(millis() - end) < 0) {
    if (mode != m) return false;
    delay(10);
  }
  return true;
}
```

---

## ビルド環境

```ini
# platformio.ini
[env:m5stack-atoms3]      # メイン WiFi 制御
build_src_filter = +<*> -<servo_test.cpp>

[env:servo_test]          # 単体サーボ確認
build_src_filter = -<*> +<servo_test.cpp>
```

### 書き込みコマンド

```bash
# メイン（WiFi制御）
pio run -e m5stack-atoms3 -t upload

# サーボ単体テスト
pio run -e servo_test -t upload
```

---

## 操作手順

### 起動

1. ロボットに電源を入れる
2. ATOM S3 の画面に `IP: 192.168.55.27` が表示されるまで待つ（約3秒）
3. iPhone の WiFi 設定で **robot0009** に接続（パスワード: `password`）
4. Safari などのブラウザで **192.168.55.27** を開く

### 操作パネル

```
[ START ]                ← 全サーボ一斉スウィープ（押すたびに START / STOP 切り替え）
[ STEP ]  [ STRETCH ]    ← 対角交互ステップ / ゆっくり全域スウィープ
[   FWD   ]              ← 前進（対角トロット歩行）
[ LEFT ]  [ RIGHT ]      ← 左右旋回
[   BACK  ]              ← 後退
[ STANDBY ] [ FOLD ]     ← 待機姿勢へ / 収納姿勢 → 脱力
[  CONFIG  ]             ← 姿勢設定パネル
```

**共通ルール**:
- 動作中に別ボタンを押すと **即座に切り替わる**
- 同じボタンを再押しすると **Standby（停止）** になる
- STANDBY / FOLD は常に単独動作（他のモードを停止）

---

## FOLD（収納モード）

FOLD ボタンの動作シーケンス：

```
1. 収納姿勢（storagePos[0..7]）へ移動（600ms）
2. 800ms 待機（サーボが到達するまで）
3. 全サーボ トルクOFF（TORQUE_ENABLE レジスタ = 0）
   → シャフトが解放され、手で動かせる状態になる
```

FOLD 中に再度 FOLD を押すか、STANDBY を押すと：

```
1. 全サーボ トルクON
2. スタンバイ姿勢（standbyPos[0..7]）へ復帰
```

> **Note**: SCS0009 はギアボックスの機械摩擦があるため、  
> トルクOFF後でも完全にフリーにはならない場合がある。  
> 収納姿勢を実際に折り畳める角度に設定しておくことが重要。

---

## CONFIG パネル（姿勢設定）

CONFIG ボタンを押すと設定パネルに切り替わる。  
**STANDBY** タブと **収納** タブを切り替えて各姿勢を調整できる。

### 操作手順

1. **CONFIG** ボタンをタップ → 設定パネルへ
2. タブを選択（STANDBY または 収納）
3. 各 ID（0〜7）の ±5 / ±20 ボタンで値を調整
   - 調整と同時にサーボが即座に動く（ライブプレビュー）
   - 値の範囲: 0〜1023（CENTER = 511 = 0度）
4. 目的の姿勢になったら **NVSに保存** をタップ
5. **戻る** でメインパネルへ

### NVS 保存の仕組み

```
起動時: NVS → standbyPos[0..7], storagePos[0..7] を読み込み
調整中: RAM のみ更新 + サーボへ即送信（NVS未変更）
「NVSに保存」押下: RAM → NVS へ書き込み（電源OFFでも保持）
```

### サーボ位置の目安

| 値 | 角度 |
|----|------|
| 408 | -30° |
| 511 | 0°（CENTER） |
| 614 | +30° |

---

## 技術メモ

### サーボグループの調整

実機のサーボ取り付け向きによって FWD/BACK/LEFT/RIGHT の動作方向が変わる。  
`main.cpp` 冒頭の配列を実機に合わせて調整すること：

```cpp
const uint8_t GA[] = {0,1,6,7};  // 対角A
const uint8_t GB[] = {2,3,4,5};  // 対角B
const uint8_t GL[] = {0,1,4,5};  // 左側
const uint8_t GR[] = {2,3,6,7};  // 右側
```

### SCS0009 TORQUE_ENABLE

```
レジスタアドレス: 40 (0x28)
値 1: トルクON（デフォルト）
値 0: トルクOFF（脱力）
```

パケット（ID=0xFE ブロードキャスト、トルクOFF）：

```
FF FF FE 04 03 28 00 D2
```

---

## ファイル構成

```
ATOMS30009/
├── README.md               ← このファイル
├── platformio.ini          ← ビルド設定（2環境）
└── src/
    ├── main.cpp            ← メイン（WiFi + WebSocket + NVS）
    └── servo_test.cpp      ← 単体サーボ確認コード（ライブラリ不使用）
```
