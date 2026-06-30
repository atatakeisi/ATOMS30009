# analysis/ — Python（変換 ＋ 将来の力学解析）

Mac mini 側のツール置き場。今は「手作り動き → 本編ボタン」の変換が中心。
将来は第2層（力学検証・自動最適化）の Jupyter/解析もここに入る（[docs/DESIGN.md](../docs/DESIGN.md)）。

## 中身

| 項目 | 内容 |
|---|---|
| `seq_to_header.py` | `sequences/*.json` → `src/motions/*.h` ＋ レジストリ `motions_all.h` を生成 |
| `sequences/` | Teaching でエクスポートした動きJSON（参照ライブラリ）。詳細は [sequences/README.md](sequences/README.md) |

## 手作り動きを本編ボタンにする手順

1. teachファーム（AP `robot0009-teach`）で姿勢を記録 → **⬇エクスポート**でJSON取得
2. JSON を `analysis/sequences/<名前>.json` に置く
   - **ファイル名＝ボタンの識別子（ASCIIのみ）**。日本語表示名はJSONの `"name"` フィールド
3. 変換:
   ```bash
   python3 analysis/seq_to_header.py
   ```
4. 本番ビルド＆書き込み → 操作ビューに新しい動作ボタンが増える
   ```bash
   pio run -e m5stack-atoms3 -t upload
   ```

## スピード調整

各コマの移行時間は JSON の各フレーム `"t"`[ms]。全体を遅く/速く、コマ毎に個別、いずれも可。
teach UI の「移行 ±」でも、JSON直接編集でも調整できる。
