# tools/ — ユーティリティ（本編とは別の小物）

| ディレクトリ | 内容 |
|---|---|
| `servo-id-set/` | SCS0009 の**サーボID設定**用スケッチ（独立PlatformIOプロジェクト）。新品サーボや交換時に各サーボへ ID 0..7 を書き込む |
| `servo-test/` | サーボ単体動作の確認用 `servo_test.cpp`（参照用。本編ビルドには含めない） |

`servo-id-set/` は自前の `platformio.ini` を持つ別プロジェクト。使うときはそのフォルダを
PlatformIO で開く（リポジトリ直下の `pio run` ではビルドされない）。
