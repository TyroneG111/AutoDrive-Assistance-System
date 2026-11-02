#pragma once
// 公共常量与数据结构（协议/模型尺寸均来自已实测确认的规格）
#include <cstddef>
#include <cstdint>
#include <vector>

namespace adas {

// ---------- 输入图像（UDP 帧）----------
constexpr int kImgW = 1280;
constexpr int kImgH = 720;
constexpr int kRowBytes = kImgW * 3;             // 3840
constexpr int kPktsPerRow = 3;
constexpr int kPktImgBytes = kRowBytes / kPktsPerRow;  // 1280
constexpr int kPktTotal = kPktImgBytes + 4;      // 1284
constexpr int kPktsPerFrame = kImgH * kPktsPerRow;     // 2160

// ---------- 双目融合图（FPGA 侧输出，与 720p 复用同一 UDP 端口）----------
// 每包 1344 B：
//   [0..1]  固定标记 111111_00000_00000（前 6 位 = 111111，用于与 720p 区分）
//   [2..3]  4 位帧同步码 + 2 位保留(00) + 10 位行号
//   [4..]   1340 B = 一行 RGB565 像素（670 × 2B）
// 292 行/帧，一行一包（无行内分包）。
constexpr int kFusionW = 670;
constexpr int kFusionH = 292;
constexpr int kFusionRowBytes = kFusionW * 2;          // 1340（RGB565）
constexpr int kFusionPktBytes = kFusionRowBytes;       // 一行一包
constexpr int kFusionPktTotal = kFusionPktBytes + 4;   // 1344

// ---------- 模型输入/输出 ----------
constexpr int kModelSize = 640;
constexpr int kNumAnchors = 8400;      // 80^2 + 40^2 + 20^2
constexpr int kNumClassesDet = 16;
constexpr int kNumClassesSeg = 10;
constexpr int kMaskCoeffs = 32;
constexpr int kProtoSize = 160;        // 640 / 4
constexpr int kNumLightClasses = 12;

// 图像模式（头字段 bits[9:5]，one-hot）
//   0x01 四路拼接：整帧 1280x720 是 2x2 拼接，按“主摄选择”取对应 1/4 区域；
//   0x02/0x04/0x08/0x10 单路：整帧就是该路画面，全屏识别，不裁剪；
//   其余值：不识别。
enum ImageMode : uint8_t {
  kModeFourWay = 0x01,   // 00001 四路拼接（取主摄对应的 1/4 区域）
  kModeCam1    = 0x02,   // 00010 cam1 单路全屏
  kModeCam2    = 0x04,   // 00100 cam2 单路全屏
  kModeCam3    = 0x08,   // 01000 cam3 单路全屏
  kModeCam4    = 0x10,   // 10000 cam4 单路全屏
};

// 驾驶行为（bits[4:0]，one-hot，0=无行为）
enum DriverBehavior : uint8_t {
  kBehavNone             = 0,
  kBehavBrake            = 1u << 0,
  kBehavForceLaneChange  = 1u << 1,
  kBehavRight            = 1u << 2,
  kBehavLeft             = 1u << 3,
  kBehavStraight         = 1u << 4,
};

struct PacketHeader {
  uint8_t  mode;        // 图像模式 one-hot
  uint8_t  behavior;    // 驾驶行为 one-hot
  uint8_t  frame_sync;  // 0..15
  uint8_t  intra_row;   // 0..2
  uint16_t line;        // 0..719
};

// 一帧完整图像（2.64 MiB）
struct Frame {
  uint8_t  rgb[kImgW * kImgH * 3];
  uint8_t  mode = 0;
  uint8_t  behavior = 0;
  uint8_t  frame_sync = 0;
  uint8_t  rows_done[kImgH] = {0};   // bit0..2 = 行内 3 包
  bool     complete = false;
  uint16_t missing_rows = 0;
  uint64_t index = 0;                // 输出序号
  uint64_t ts_us = 0;                // 帧结束时间（回放/UDP 时间戳）
};

// 检测/分割实例（坐标为原图像素）
struct Detection {
  float x1, y1, x2, y2;
  float score;
  int   cls;
  int   mask_idx = -1;   // 分割：在 coeffs 中的 anchor 下标（NMS 前）
};

} // namespace adas
