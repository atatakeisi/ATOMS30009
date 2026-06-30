# hardware/ — ハードウェア資料

筐体・配線・3Dプリント等のハード関連。

## 構成

| ディレクトリ | 内容 |
|---|---|
| `3d-models/` | 3Dプリント用データ（`.stl` / `.3mf`）。**現在は作業中のため未コミット**（後日追加） |

## 主要ハード仕様（要点。詳細は [docs/DESIGN.md](../docs/DESIGN.md)）

- マイコン: M5Stack ATOM S3（ESP32-S3, 128x128 LCD, ボタン=GPIO41）
- サーボ: SCS0009 ×8（半二重UART 1Mbps）
  - TX=GPIO5（1kΩ直列）／ RX=GPIO6（バス直結）
  - ⚠️ **GPIO38/39 は内蔵IMU(MPU6886)のI2Cのため使用禁止**
- 電源: バッテリー駆動。**筐体組込時はUSB接続不可**（USBとバッテリー排他）
  → 運用時の入出力は LCD と WiFi のみ

## 3Dモデルの追加（後日）

`.stl`/`.3mf` を `hardware/3d-models/` に置く。`.gitignore` で当面コミット対象外に
しているので、確定したら `.gitignore` の該当行を外してコミットする。
