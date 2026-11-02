#pragma once
#include "common/types.h"

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>

namespace adas {

// 按 (帧同步码, 行号, 行内同步) 重组整帧；整帧齐备后才交付。
class FrameAssembler {
public:
  using Callback = std::function<void(const Frame&)>;

  FrameAssembler();
  ~FrameAssembler();

  void set_callback(Callback cb) { cb_ = std::move(cb); }

  // 容忍算法（运行中可切换，内部用 atomic 保存）：
  //   开：缺的行用上一帧同一行补齐，缺行数 <= kTolMaxRows 时仍交付（complete=true，
  //       但 Frame::missing_rows > 0 标记为“补过行”）；
  //   关：原行为——720 行必须全齐，否则 complete=false 由上层丢弃。
  static constexpr int kTolMaxRows = kImgH * 5 / 100;   // 36 行（5%）
  void set_tolerance(bool on) { tol_.store(on); }
  bool tolerance() const { return tol_.load(); }

  // 推入一个 UDP payload（含 4B 头）；返回是否接受
  bool push(const uint8_t* payload, size_t len, uint64_t ts_us = 0);

  // 结束（提交最后一个帧，可能不完整）
  void flush(uint64_t ts_us = 0);

  uint64_t packets() const { return packets_; }
  uint64_t frames_out() const { return frames_out_; }
  uint64_t frames_incomplete() const { return frames_incomplete_; }

private:
  void start_(uint8_t fsync, uint64_t ts_us);
  void finalize_(uint64_t ts_us);

  std::unique_ptr<Frame> frames_[2];
  int      cur_ = 0;
  bool     started_ = false;
  bool     prev_valid_ = false;   // 双缓冲里另一块是否真的装着上一帧
  std::atomic<bool> tol_{false};
  uint16_t rows_full_ = 0;
  uint64_t idx_ = 0;
  uint64_t ts_start_ = 0;

  Callback cb_;
  uint64_t packets_ = 0;
  uint64_t frames_out_ = 0;
  uint64_t frames_incomplete_ = 0;
};

} // namespace adas
