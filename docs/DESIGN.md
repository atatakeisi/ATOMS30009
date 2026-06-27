# ATOMS30009 進化プロジェクト — アーキテクチャ設計とロードマップ

対象: M5Stack ATOM S3 (ESP32-S3) + SCS0009 ×8 四足歩行ロボット
本書は実装に先立つ設計合意。コードは本書のフェーズ順に追加する。

---

## ビジョン

> 手作業で作った動きを起点に、計算で精査・最適化し、自律的に安定して動ける
> ロボットに育てる。**人間の直感 × 計算機の精査**のループを回す。

3層構造で実現する:

1. **第1層 キーフレームTeaching**（ESP32）: パラパラ漫画方式で手作業の動きを作る
2. **第2層 力学検証・自動最適化**（Python/Mac mini）: 実走データを数値評価し最適化
3. **第3層 自律判断**（将来）: IMUから状況判断し動作を自動選択

---

## ★ 最重要の設計原則：形（幾何）と力学を分離する

関節角（raw[8]）と「その瞬間にかかっている力学（重力）」を同じデータに混ぜると、
動いた瞬間に状態の関係性が発散して扱えなくなる。**この2つは別々に扱う。**

| 量 | 必要な情報 | 重力依存 | raw[8]だけで足りるか | 取得場所 |
|---|---|---|---|---|
| **幾何（キネマティクス）**<br>脚先軌跡・関節角速度/加速度 | 関節角 = raw[8] | しない（ボディ座標相対） | **足りる** | Teaching（横倒し・torque OFF） |
| **力学・安定性**<br>ZMP・支持多角形・転倒余裕 | 関節角 ＋ 重力方向(ボディ姿勢) ＋ 実接地 | **する** | **足りない** | 計測再生（立位・torque ON） |

### 帰結

- **Teaching中（横倒し・torque OFF）のIMUは無意味**（横倒し姿勢を示すだけで、
  立位再生時の重力状態と対応しない）。Teachingは **raw[8] だけ記録**する。
- 重力情報が意味を持つのは **実際に床に立たせて torque ON で動かしている瞬間だけ**。
  よって安定性検証の入力は「Teachingキーフレーム＋IMU」ではなく、
  **「計測再生で取得した raw時系列 ＋ IMU時系列 ＋ 実測raw」**とする。
- **役割分担**: Teaching = 動きの「形」をオーサリング（重力非依存） /
  計測再生(Logging) = 重力下の力学データを取得。

---

## データの流れ

```
        [手で関節を動かす]
               │ (torque OFF, 横倒し)
               ▼
   ┌──────────────────────┐  read raw[8] (0x38)
   │   ESP32-S3 (ATOM S3) │◀──────────────  SCS0009 ×8 (half-duplex 1Mbps)
   │  Teaching firmware    │──────────────▶  write pos(0x2A) / torque(0x28)
   └──────────────────────┘
       │  ▲           │  ▲
 POST  │  │ GET JSON  │  │ putBytes/getBytes
 (cmd) │  │ (status)  │  │
       ▼  │           ▼  │
   ┌──────────┐  ┌──────────┐  ┌──────────┐
   │ Browser  │  │ LittleFS │  │   NVS    │
   │ (Mac)    │  │ /seq/*.json  │ offset[],│
   └──────────┘  │ シーケンス│  │ period等 │
                 └──────────┘  └──────────┘

   [計測再生 Logging] 立位・torque ON で再生しつつ時刻同期で記録:
     ・指令 raw[8]（commanded）
     ・実測 raw[8]（present position 読み戻し）← 追従誤差＝負荷の代理指標
     ・IMU theta_X/Y, accel, gyro          ← ここで初めて重力が「適応中」になる
               │ GET /log/export (自己完結JSON: ↑時系列 + offset + L1/L2 + 符号)
               ▼
   ┌──────────────────────┐
   │  Mac mini / Python   │ raw→IK逆変換→軌跡/速度/加速度→安定性評価→最適化
   │  Jupyter + numpy/scipy│
   └──────────────────────┘
               │ optimized params (JSON)
               ▼ POST /params/import → NVS
```

シーケンスは **raw値で記録**（較正非依存でそのまま再生可能）。Python解析に要る
`offset[]`・`L1/L2`・IK符号規則は **export JSONに同梱**して Python側を自己完結にする。

---

## プロジェクト構成（単一リポジトリ・多env）

```
ATOMS30009/
├─ platformio.ini          # env: m5stack-atoms3(prod, 現行) / m5stack-atoms3-teach(新規)
├─ src/
│  ├─ main.cpp             # ← 現行 prod。Phase1〜4では一切触らない
│  └─ teach/
│     └─ main_teach.cpp    # Teaching/計測再生ファーム（独立 env）
├─ data/                   # LittleFS イメージ（既定シーケンス等・Phase2以降）
├─ analysis/               # Python（Mac mini, Phase4）
│  ├─ requirements.txt
│  ├─ robot/               # ik.py / zmp.py / io_http.py / schema.py
│  └─ notebooks/
├─ docs/
│  ├─ DESIGN.md            # 本書
│  └─ schema/keyframe.schema.json
└─ shared/protocol.md      # SCS0009 プロトコルの単一情報源
```

`build_src_filter` で prod は `src/main.cpp`、teach は `src/teach/` のみをビルド。
**現行ファームのビルドは無改変**で温存する。

### コード重複の方針

Phase 1〜4 は **prod に触れないため、SCS/IK/offset 等のコードは teach 側に重複**する。
ドリフトは次で抑制する:
- 較正(offset) は **同一 NVS 名前空間 "parameter" を共有**（データは二重化しない）
- プロトコルは `shared/protocol.md` を単一情報源にする
- Teaching 検証完了後、**Phase 5 で `lib/` 抽出して一本化**（その時に prod も移行）

---

## 実装ロードマップ

| Phase | 内容 | 難易度 | 工数感 | 現行統合 |
|---|---|---|---|---|
| **0** 準備・検証 | teach env・`M5.BtnA`確認・**torque OFFで位置READ可否を実機検証**・無ジャークtorque ON手順確認 | 低 | 半日 | 分離 |
| **1** Teaching中核 | torque off/on・raw[8]読取・RAM記録・ボタンUX・Teaching画面(ポーリング)・Linear再生 | 中 | 2〜3日 | 分離 |
| **1.5** 計測再生(Logging) | 立位・torque ONで再生しつつ commanded/present raw + IMU を時刻同期ロギング・`/log/export` | 中 | 1〜2日 | 分離 |
| **2** 永続化 | LittleFSへ名前付き保存/一覧/削除/リネーム・`/teach/export` JSON・スキーマ凍結 | 低〜中 | 1〜2日 | 分離 |
| **3** 補間強化 | Cosine→Catmull-Rom・区間ごと時間・クランプ | 低 | 1日 | 分離 |
| **4** Python解析 | JSON取込→IK逆変換→軌跡/速度/加速度→静的安定マージン→ZMP近似/IMU同期→最適化→POST反映 | 中〜高 | 段階的 | 分離 |
| **5** ライブラリ化・統合 | `lib/`へSCS/IK/Keyframe抽出、両ファーム共有、再生を本番UIの動作ボタンへ追加 | 中 | 2〜3日 | **ここで統合** |
| **6** 自律(将来) | 転倒検知→自動起き上がり・歩行中アクティブ平衡・立て直し・脚負荷推定 | 高 | 継続 | 統合済前提 |

**優先順位の根拠**: Teaching(人間の直感)を確立 → 計測再生でその動きを重力下で実測 →
そのログが Phase4(計算機の精査)の入力。JSONスキーマは Phase2 で凍結してから Python着手。

---

## 技術選定と根拠

### 保存先: NVS(既存) + LittleFS(新規シーケンス)
- **NVS**: キー値。較正offset・単一パラメータに最適。可変長・名前付き・複数の
  シーケンス管理は苦手。既存資産なので**移動しない**。
- **LittleFS**: 実FS・ウェアレベリング・ディレクトリ列挙。`/seq/walk.json` で
  **名前付き複数保存/一覧/追加削除**が自然。8MBフラッシュに余裕。**採用**。
- **SPIFFS**: 非推奨(ディレクトリ無し・遅い)。除外。
- 注意: 既定パーティションのFS領域を Phase2 で確認し、`board_build.filesystem=littlefs`
  と適切な partition scheme(FSに1〜2MB) を設定。

### WebUIリアルタイム: 短間隔ポーリング
- 現行は同期型 WebServer + POST。SSEは単一コネクション占有で相性悪く、WebSocketは
  ESPAsyncWebServer等への移行で現行POST構造を壊すリスク大。
- Teaching/Logging表示要件は数Hzで十分 → `GET /teach/status`(副作用なし、先読み安全)を
  JS `fetch` で200〜300ms間隔ポーリング。**新ライブラリ0・最も堅牢**。**採用**。

### 補間: Linear → Cosine → Catmull-Rom
- **Linear**: 最単純・検証向き。コマ境界で速度不連続→ガクつき・負荷。
- **Cosine** `0.5-0.5cos(πt)`: 各区間ease-in-out、各コマで速度0=「ポーズto ポーズ」最適。
  計算極小。**本番標準**。
- **Catmull-Rom**: コマを止まらず通過=連続歩容向き。オーバーシュート→**必ずクランプ**。Phase3後半。
- いずれも `softStartToHorizontal()` と同じ「ステップ外/サーボ内・80Hzペーシング」で
  中間setpoint生成、SCS内蔵速度制御と併用。

### Python環境
- Python 3.11+/venv、`numpy scipy matplotlib pandas jupyterlab requests`、
  日本語は `japanize-matplotlib`。
- `robot/` に ESP32 と同一の IK/符号を移植（export JSON同梱値を使い二重管理回避）。

---

## リスクと注意点

1. **torque OFF時の自重崩落(最重要・安全)**: 立位でトルクを切ると脱力して倒れる。
   Teaching開始は**横に寝かせる/支える前提**。UIに⚠️警告。
2. **トルク再投入時のジャーク**: OFF中はゴールregisterが古い→ONで旧ゴールへスナップ。
   **「現在位置READ→ゴール書込→torque ON」**の順で無ジャーク化。(*Phase0で実機確認*)
3. **torque OFFでの位置READ可否**: Teaching全体の前提。**Phase0で必ず実機確認**。
4. **半二重UART占有**: Teaching=READのみ/再生=WRITEのみで競合させない。READを80Hzで
   回さない(8軸ラウンドトリップでバス飽和)。表示用は数Hz。自己エコー対策は既存 `scs_readPosition` 流用。
5. **GPIO制約**: `GPIO38/39`(IMU I2C)は**絶対禁止**。ボタンは`GPIO41`=`M5.BtnA`。
   現行loop()は`M5.update()`未呼出 → teachファームではループ先頭で`M5.update()`必須。
6. **リソース**: 現状 RAM 14.9% / Flash 28.7% と余裕大。1コマ=16B×コマ数(64コマ≒1KB)。
7. **現行との共存/統合(Phase5)**: 重複は較正NVS共有＋protocol.md単一情報源で抑制。
   **「動くものを壊さない」最優先**で分離→検証後に lib/ 抽出。
8. **USBとサーボ電源が排他(重要)**: ATOM S3 を筐体に組むと **USB接続不可**
   （バッテリーとの同時接続回避のため）。→ USB接続時はバッテリー非接続=サーボ電源なし=
   位置READ不可、という逆転が起きる。**運用時の入出力は LCD と WiFi のみ**で完結させ、
   **Serial(USB)はデバッグ補助に留める**。検証用フィードバックは必ずLCD/WiFiに出す。

---

## Phase 0 検証ファーム仕様（`src/teach/main_teach.cpp` 初版）

Teaching全体の前提（torque OFF中のREAD、無ジャークtorque ON）を実機で潰すための
最小ファーム。`m5stack-atoms3-teach` env でビルド・書き込みして検証する。

### ボタン操作（GPIO41 = `M5.BtnA`）
| 操作 | API | 動作 |
|---|---|---|
| 短押し | `wasClicked()` | 全8サーボの present position をREADしSerial/LCDへ出力 |
| 長押し(>800ms) | `wasHold()` | 全8サーボ torque OFF（脱力）|
| ダブルクリック | `wasDoubleClicked()` | 無ジャーク torque ON（現在位置READ→ゴール書込→enable）|

### 検証手順
1. teach env を書き込み、床に寝かせる。
2. 長押し → torque OFF。手で関節を動かす。
3. 短押し → 出力された raw が手の動きに追従して変化するか確認
   （＝**torque OFF中にREADできる**ことの確認）。
4. ダブルクリック → ジャークなく現在姿勢で保持に入るか確認。

この4点が通れば Phase 1 へ進む。

---

## データフォーマット定義

オンデバイス（RAM/再生・コンパクト）:
```c
struct Keyframe { uint16_t raw[8]; };            // 16B
struct Sequence {
  char     name[16];
  uint16_t frameCount;
  uint16_t transMs;        // 既定のコマ間移行(ms)
  uint8_t  interp;         // 0=Linear 1=Cosine 2=Catmull
  Keyframe frames[MAX_FRAMES];   // 例: 64
};
```

ディスク/Python交換（LittleFS `/seq/<name>.json`・自己完結）:
```json
{
  "name": "wave", "transMs": 600, "interp": "cosine",
  "calib": { "offset": [-35,0,13,-7,53,-44,15,34], "L1": 50.0, "L2": 65.0,
             "sign": "RL/FR:+ , RR/FL:-" },
  "frames": [ {"raw":[511,511,524,504,564,467,526,545]} ]
}
```

計測再生ログ（Phase1.5・`/log/export`・自己完結）:
```json
{
  "seq": "wave", "interp": "cosine", "dt_ms": 12,
  "calib": { "offset": [...], "L1":50.0, "L2":65.0, "sign":"..." },
  "samples": [
    {"t":0,   "cmd":[...], "present":[...], "theta":[0.1,-0.3], "acc":[..], "gyro":[..]},
    {"t":12,  "cmd":[...], "present":[...], "theta":[...],       "acc":[..], "gyro":[..]}
  ]
}
```

---

## SCS0009 プロトコル要約（詳細は shared/protocol.md）

- 位置書込: `FF FF ID 09 03 2A posH posL 00 00 spdH spdL CS`（pos/spd はビッグエンディアン）
- 位置読取要求: `FF FF ID 04 02 38 02 CS`
- 位置読取応答: `FF FF ID 04 err posH posL CS`
  （★**上位バイト先＝big-endian**。WRITEと同じ並び。実機ダンプで確定。
  プロンプト元仕様の「low byte first」は誤りだったので注意）
- 半二重の自己エコー: 送信8byteが必ずRXに返る。READ要求のエコーは応答用checksum式を
  偶然満たし `0x238=568` を返すため、**送信後に8byteをバイト数で読み捨ててから**応答を探す
  （ヘッダ探索＋checksum＋位置range[0,1023]検証で確定）
- torque: `FF FF ID 04 03 28 EN CS`（0x28=Torque Enable, EN=0:OFF / 1:ON）
- checksum: `~(ID + LEN + 以降バイト合計) & 0xFF`
- 半二重シングルワイヤ: 送信8byteが自RXに自己エコー → ヘッダ探索＋checksum検証で除去
