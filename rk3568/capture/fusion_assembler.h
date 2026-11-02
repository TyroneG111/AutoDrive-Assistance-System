#pragma once
// 双目融合图（FPGA 侧输出，670×292 RGB565）的 UDP 重组。
// 与 720p 图像共用同一端口，靠报文前 6 位区分（见 capture/packet.h）。
#include "common/types.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace adas {

// 一帧融合图（RGB565，行优先）
struct FusionFrame {
  uint16_t px[kFusionW * kFusionH] = {};
  uint8_t  frame_sync = 0;
  uint16_t missing_rows = 0;   // >0 表示这一帧有行是补出来的
  bool     complete = false;
  uint64_t index = 0;
  uint64_t ts_us = 0;
};

// 一行一包，共 kFusionH 包/帧。
class FusionAssembler {
public:
  using Callback = std::function<void(const FusionFrame&)>;

  FusionAssembler();
  ~FusionAssembler();

  void set_callback(Callback cb) { cb_ = std::move(cb); }

  // 容忍算法：缺行 <= kTolMaxRows 的帧一律交付；
  // 开关只决定缺行怎么填：开=用上一帧同位置行补齐（画面无断裂），关=填黑。
  // 30%（≈87 行）：实测 720p 大流量与融合图同口时 CPU/软中断吃满，融合包会成批丢
  // （单帧缺 80~115 行），预算太小会把帧几乎全判为不可用，画面就停在第一帧不动。
  static constexpr int kTolMaxRows = kFusionH * 30 / 100;   // 87 行（30%）
  void set_tolerance(bool on) { tol_.store(on); }
  bool tolerance() const { return tol_.load(); }

  // payload 含 4B 头（1344B）；接受返回 true
  bool push(const uint8_t* payload, size_t len, uint64_t ts_us = 0);
  void flush(uint64_t ts_us = 0);

  uint64_t packets() const { return packets_; }
  uint64_t frames_out() const { return frames_out_; }
  uint64_t frames_incomplete() const { return frames_incomplete_; }

private:
  void start_(uint8_t fsync, uint64_t ts_us);
  void finalize_(uint64_t ts_us);

  std::unique_ptr<FusionFrame> frames_[2];
  int      cur_ = 0;
  bool     started_ = false;
  bool     prev_valid_ = false;
  std::atomic<bool> tol_{false};
  uint8_t  row_done_[kFusionH] = {};
  uint16_t rows_full_ = 0;
  uint64_t idx_ = 0;
  uint64_t ts_start_ = 0;

  Callback cb_;
  uint64_t packets_ = 0;
  uint64_t frames_out_ = 0;
  uint64_t frames_incomplete_ = 0;
};

// RGB565 → RGB888（out 需要 kFusionW*kFusionH*3 字节）。
// big_endian：FPGA 发的是 0xRRRRRGGGGGGBBBBB 的高字节在前时为 true。
// 若发现红蓝/高低位错乱，把 PipelineConfig::fusion565_bigendian 置 false 即可。
void fusion_to_rgb888(const FusionFrame& f, uint8_t* out, bool big_endian = true);

} // namespace adas
