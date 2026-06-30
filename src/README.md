# src/ — ファームウェア

PlatformIO はファームを `src/` 配下に置く規約。ここを役割で3つに分ける。
ビルド対象は [platformio.ini](../platformio.ini) の `build_src_filter` で env ごとに切替。

| ディレクトリ | env | 内容 |
|---|---|---|
| `prod/` | `m5stack-atoms3` | **本番（筐体専用）ファーム**。WebUIで歩行・旋回・IMU・Teaching動作などを操作 |
| `teach/` | `m5stack-atoms3-teach` | **Teaching工房ファーム**。手で姿勢を作りコマ記録→再生→JSONエクスポート |
| `motions/` | （ヘッダのみ） | **手作り動きの自動生成ヘッダ**。`prod` が相対includeで取り込む |

## ビルド／書き込み

```bash
pio run -e m5stack-atoms3        -t upload   # 本番（AP: robot0009）
pio run -e m5stack-atoms3-teach  -t upload   # Teaching（AP: robot0009-teach）
```

## motions/ について

`motions/*.h` は手で編集しない。[analysis/seq_to_header.py](../analysis/seq_to_header.py) が
`analysis/sequences/*.json` から生成する。`motion.h`（型定義）だけ手書き。
動きの追加手順は [analysis/README.md](../analysis/README.md) 参照。

## 設計の全体像

3層アーキテクチャ・プロトコル・サーボマップ等は [docs/DESIGN.md](../docs/DESIGN.md)。
