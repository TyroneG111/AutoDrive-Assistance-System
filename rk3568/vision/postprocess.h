#pragma once
#include "common/types.h"
#include "vision/preprocess.h"

#include <vector>

namespace adas {

struct DetParams {
  float conf_light = 0.20f;   // 检测：灯类（0..11）
  float conf_def   = 0.25f;   // 检测：其他类 / 分割统一
  // 可选的逐类阈值数组（长度为 nc）。非空时优先于 conf_light / conf_def。
  const float* per_class = nullptr;
  float iou        = 0.45f;
};

// boxes : [4][8400]  (cx,cy,w,h 已归一化 0..1)
// scores: [nc][8400] (已过 sigmoid)
// multi_label=true: 每个 anchor 的每个类别独立判断（与 ultralytics 默认一致）；
//                   为 false 时每个 anchor 只取 argmax 一个类（旧行为，会丢重叠类别）。
std::vector<Detection> decode_nms(const float* boxes, const float* scores, int nc, int nlight,
                                  const DetParams& p, const PreprocessMeta& m,
                                  bool multi_label = false);

// 掩码：out160[j] = sigmoid( Σ_c coeffs[c*8400+idx] · protos[c*160*160+j] )
void compute_mask160(const float* coeffs, int idx, const float* protos, float* out160);

} // namespace adas
