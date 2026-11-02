#include "capture/fusion_assembler.h"

#include "capture/packet.h"

#include <cstring>

namespace adas {

FusionAssembler::FusionAssembler() {
  frames_[0] = std::make_unique<FusionFrame>();
  frames_[1] = std::make_unique<FusionFrame>();
}

FusionAssembler::~FusionAssembler() = default;

bool FusionAssembler::push(const uint8_t* payload, size_t len, uint64_t ts_us) {
  FusionHeader h;
  if (!parse_fusion_header(payload, len, h)) return false;
  ++packets_;

  if (!started_ || h.frame_sync != frames_[cur_]->frame_sync) {
    if (started_) finalize_(ts_us);
    start_(h.frame_sync, ts_us);
  }

  FusionFrame& f = *frames_[cur_];
  // 第 [4..] 字节就是一行 RGB565（1340B），整行直拷
  std::memcpy(f.px + static_cast<size_t>(h.line) * kFusionW, payload + 4, kFusionRowBytes);
  if (!row_done_[h.line]) {
    row_done_[h.line] = 1;
    ++rows_full_;
  }
  return true;
}

void FusionAssembler::flush(uint64_t ts_us) {
  if (started_) finalize_(ts_us);
}

void FusionAssembler::start_(uint8_t fsync, uint64_t ts_us) {
  FusionFrame& f = *frames_[cur_];
  f.frame_sync = fsync;
  f.complete = false;
  f.missing_rows = 0;
  f.index = 0;
  f.ts_us = 0;
  std::memset(row_done_, 0, sizeof(row_done_));
  rows_full_ = 0;
  started_ = true;
  ts_start_ = ts_us;
}

void FusionAssembler::finalize_(uint64_t ts_us) {
  FusionFrame& f = *frames_[cur_];
  const FusionFrame& prev = *frames_[1 - cur_];   // 双缓冲的另一块 = 上一帧
  const bool tol = tol_.load();

  uint16_t missing = 0;
  for (int y = 0; y < kFusionH; ++y) {
    if (row_done_[y]) continue;
    ++missing;
    uint16_t* dst = f.px + static_cast<size_t>(y) * kFusionW;
    if (tol && prev_valid_)
      std::memcpy(dst, prev.px + static_cast<size_t>(y) * kFusionW, kFusionRowBytes);
    else
      std::memset(dst, 0, kFusionRowBytes);
  }
  f.missing_rows = missing;
  // 交付判定只看「缺行是否超过 5%」：少一两行也照常显示（否则一帧都出不来）。
  // 容忍开关只决定缺行怎么处理：开=用上一帧同位置补齐；关=填黑。
  f.complete = (missing <= kTolMaxRows);

  f.index = idx_++;
  f.ts_us = ts_us ? ts_us : ts_start_;

  prev_valid_ = true;
  ++frames_out_;
  if (!f.complete) ++frames_incomplete_;
  if (cb_) cb_(f);

  cur_ = 1 - cur_;
  started_ = false;
}

void fusion_to_rgb888(const FusionFrame& f, uint8_t* out, bool big_endian) {
  if (!out) return;
  // f.px 是 memcpy 进来的原始两字节，在小端主机上等价于 (b0 | b1<<8)。
  // 若 FPGA 按大端发（高字节在前），需要做一次字节交换才是正确的 565 值。
  const size_t n = static_cast<size_t>(kFusionW) * kFusionH;
  for (size_t i = 0; i < n; ++i) {
    uint16_t v = f.px[i];
    if (big_endian) v = static_cast<uint16_t>((v >> 8) | (v << 8));
    // 位扩展：5/6/5 位按比例铺满 8 位（r5<<3 | r5>>2）
    const uint8_t r5 = static_cast<uint8_t>((v >> 11) & 0x1F);
    const uint8_t g6 = static_cast<uint8_t>((v >> 5) & 0x3F);
    const uint8_t b5 = static_cast<uint8_t>(v & 0x1F);
    out[i * 3 + 0] = static_cast<uint8_t>((r5 << 3) | (r5 >> 2));
    out[i * 3 + 1] = static_cast<uint8_t>((g6 << 2) | (g6 >> 4));
    out[i * 3 + 2] = static_cast<uint8_t>((b5 << 3) | (b5 >> 2));
  }
}

} // namespace adas
