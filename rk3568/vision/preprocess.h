#pragma once
#include <cstddef>
#include <cstdint>

namespace adas {

struct PreprocessMeta {
  float scale = 1.f;   // 原图 -> 640 的缩放比
  int   pad_x = 0;
  int   pad_y = 0;
  int   src_w = 0;
  int   src_h = 0;
};

// 逐通道色彩归一化与 BGR 交换已移除（模型训练时未使用）

// RGB888 -> 640x640 NHWC uint8（保持 0..255，不做 /255）
// 注意：RKNN 内嵌 mean/std 归一化时**只接受 NHWC 源布局**，
//       若传 NCHW 会报 "unsupported src layout for normalize"。
bool preprocess(const uint8_t* rgb, int w, int h, uint8_t* out_hwc, PreprocessMeta* meta);

} // namespace adas
