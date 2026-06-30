#pragma once
// Teaching動作の型定義（手書き）。生成ヘッダ(<name>.h / motions_all.h)はこれをincludeする。
// データは raw[8]（ID0..7の生サーボ位置 0..1023）と、そのコマへ移行する時間 toMs[ms]。
// raw値は較正非依存でそのまま再生可能（playMotionがscs_moveToPosへ直接渡す）。
#include <stdint.h>

struct MotionFrame {
  uint16_t raw[8];
  uint16_t toMs;     // このコマへ（前コマ or 現在位置から）移行する時間[ms]
};

struct Motion {
  const char* name;
  const MotionFrame* frames;
  uint8_t count;
};
