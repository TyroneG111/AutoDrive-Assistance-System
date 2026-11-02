#include "capture/frame_assembler.h"
#include "capture/packet.h"

#include <cstring>

namespace adas {

FrameAssembler::FrameAssembler() {
  frames_[0] = std::make_unique<Frame>();
  frames_[1] = std::make_unique<Frame>();
}

FrameAssembler::~FrameAssembler() = default;

bool FrameAssembler::push(const uint8_t* payload, size_t len, uint64_t ts_us) {
  PacketHeader h;
  if (!parse_header(payload, len, h)) return false;
  ++packets_;

  if (!started_ || h.frame_sync != frames_[cur_]->frame_sync) {
    if (started_) finalize_(ts_us);
    start_(h.frame_sync, ts_us);
  }

  Frame& f = *frames_[cur_];
  f.mode = h.mode;
  f.behavior = h.behavior;

  uint8_t* dst = f.rgb + static_cast<size_t>(h.line) * kRowBytes +
                 static_cast<size_t>(h.intra_row) * kPktImgBytes;
  std::memcpy(dst, payload + 4, kPktImgBytes);

  const uint8_t bit = static_cast<uint8_t>(1u << h.intra_row);
  if ((f.rows_done[h.line] & bit) == 0) {
    f.rows_done[h.line] |= bit;
    if (f.rows_done[h.line] == 0x07) ++rows_full_;
  }
  return true;
}

void FrameAssembler::flush(uint64_t ts_us) {
  if (started_) finalize_(ts_us);
}

void FrameAssembler::start_(uint8_t fsync, uint64_t ts_us) {
  Frame& f = *frames_[cur_];
  f.frame_sync = fsync;
  f.complete = false;
  f.missing_rows = 0;
  f.index = 0;
  f.ts_us = 0;
  std::memset(f.rows_done, 0, sizeof(f.rows_done));
  rows_full_ = 0;
  started_ = true;
  ts_start_ = ts_us;
}

void FrameAssembler::finalize_(uint64_t ts_us) {
  Frame& f = *frames_[cur_];
  // 双缓冲：另一块此刻还装着上一帧（本帧只写过 cur_），可直接取来补行
  const Frame& prev = *frames_[1 - cur_];
  const bool tol = tol_.load();

  uint16_t missing = 0;
  for (int y = 0; y < kImgH; ++y) {
    if (f.rows_done[y] == 0x07) continue;
    ++missing;
    if (tol && prev_valid_) {
      // 用上一帧同一行补齐：只差一个帧周期（33ms），肉眼几乎不可见；
      // 相比清零成黑行，它不会把车道线/掩膜打断，对后续聚类更友好。
      std::memcpy(f.rgb + static_cast<size_t>(y) * kRowBytes,
                  prev.rgb + static_cast<size_t>(y) * kRowBytes, kRowBytes);
    } else {
      // 缺行清零，避免残留上一帧像素
      std::memset(f.rgb + static_cast<size_t>(y) * kRowBytes, 0, kRowBytes);
    }
  }
  f.missing_rows = missing;

  if (tol)
    f.complete = (missing <= kTolMaxRows);          // 容忍模式下缺行仍算可用帧
  else
    f.complete = (rows_full_ == kImgH);             // 原行为：一行不齐就丢

  f.index = idx_++;
  f.ts_us = ts_us ? ts_us : ts_start_;

  prev_valid_ = true;
  ++frames_out_;
  if (!f.complete) ++frames_incomplete_;
  if (cb_) cb_(f);

  cur_ = 1 - cur_;
  started_ = false;
}

} // namespace adas
