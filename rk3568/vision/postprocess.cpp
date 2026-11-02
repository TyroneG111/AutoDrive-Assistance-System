#include "vision/postprocess.h"

#include <algorithm>
#include <cmath>

namespace adas {
namespace {

inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

float iou_of(const Detection& a, const Detection& b) {
  const float x1 = std::max(a.x1, b.x1);
  const float y1 = std::max(a.y1, b.y1);
  const float x2 = std::min(a.x2, b.x2);
  const float y2 = std::min(a.y2, b.y2);
  const float w = std::max(0.f, x2 - x1);
  const float h = std::max(0.f, y2 - y1);
  const float inter = w * h;
  const float ua = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return ua > 0.f ? inter / ua : 0.f;
}

} // namespace

std::vector<Detection> decode_nms(const float* boxes, const float* scores, int nc, int nlight,
                                  const DetParams& p, const PreprocessMeta& m,
                                  bool multi_label) {
  std::vector<Detection> dets;
  dets.reserve(512);

  const float* bx = boxes + 0 * kNumAnchors;
  const float* by = boxes + 1 * kNumAnchors;
  const float* bw = boxes + 2 * kNumAnchors;
  const float* bh = boxes + 3 * kNumAnchors;

  const float inv_scale = (m.scale > 1e-6f) ? 1.f / m.scale : 1.f;
  const int kMaxCand = 300;   // 与 ultralytics max_det 对齐

  for (int a = 0; a < kNumAnchors; ++a) {
    if (multi_label) {
      // 每个类别独立判断（同一 anchor 可同时属于多个类）
      for (int c = 0; c < nc; ++c) {
        const float s = scores[c * kNumAnchors + a];
        const float thr = p.per_class ? p.per_class[c]
                                      : ((c < nlight) ? p.conf_light : p.conf_def);
        if (s < thr) continue;

        const float cx = bx[a] * kModelSize;
        const float cy_a = by[a] * kModelSize;
        const float w  = bw[a] * kModelSize;
        const float h  = bh[a] * kModelSize;
        float x1 = cx - w * 0.5f, y1 = cy_a - h * 0.5f;
        float x2 = cx + w * 0.5f, y2 = cy_a + h * 0.5f;
        x1 = (x1 - m.pad_x) * inv_scale;
        x2 = (x2 - m.pad_x) * inv_scale;
        y1 = (y1 - m.pad_y) * inv_scale;
        y2 = (y2 - m.pad_y) * inv_scale;
        if (m.src_w > 0 && m.src_h > 0) {
          x1 = clampf(x1, 0.f, static_cast<float>(m.src_w - 1));
          x2 = clampf(x2, 0.f, static_cast<float>(m.src_w - 1));
          y1 = clampf(y1, 0.f, static_cast<float>(m.src_h - 1));
          y2 = clampf(y2, 0.f, static_cast<float>(m.src_h - 1));
        }
        if (x2 <= x1 || y2 <= y1) continue;

        Detection d{};
        d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;
        d.score = s;
        d.cls = c;
        d.mask_idx = a;
        dets.push_back(d);
        if (static_cast<int>(dets.size()) >= kMaxCand) break;
      }
      if (static_cast<int>(dets.size()) >= kMaxCand) break;
      continue;
    }

    int best = -1;
    float bs = 0.f;
    for (int c = 0; c < nc; ++c) {
      const float s = scores[c * kNumAnchors + a];
      if (s > bs) { bs = s; best = c; }
    }
    if (best < 0) continue;
    const float thr = p.per_class ? p.per_class[best]
                                  : ((best < nlight) ? p.conf_light : p.conf_def);
    if (bs < thr) continue;

    const float cx = bx[a] * kModelSize;
    const float cy = by[a] * kModelSize;
    const float w  = bw[a] * kModelSize;
    const float h  = bh[a] * kModelSize;

    float x1 = cx - w * 0.5f, y1 = cy - h * 0.5f;
    float x2 = cx + w * 0.5f, y2 = cy + h * 0.5f;

    // 反 letterbox
    x1 = (x1 - m.pad_x) * inv_scale;
    x2 = (x2 - m.pad_x) * inv_scale;
    y1 = (y1 - m.pad_y) * inv_scale;
    y2 = (y2 - m.pad_y) * inv_scale;

    if (m.src_w > 0 && m.src_h > 0) {
      x1 = clampf(x1, 0.f, static_cast<float>(m.src_w - 1));
      x2 = clampf(x2, 0.f, static_cast<float>(m.src_w - 1));
      y1 = clampf(y1, 0.f, static_cast<float>(m.src_h - 1));
      y2 = clampf(y2, 0.f, static_cast<float>(m.src_h - 1));
    }
    if (x2 <= x1 || y2 <= y1) continue;

    Detection d{};
    d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;
    d.score = bs;
    d.cls = best;
    d.mask_idx = a;
    dets.push_back(d);
  }

  // 类别内 NMS
  std::sort(dets.begin(), dets.end(),
            [](const Detection& a, const Detection& b) { return a.score > b.score; });

  std::vector<char> removed(dets.size(), 0);
  std::vector<Detection> keep;
  keep.reserve(dets.size());
  for (size_t i = 0; i < dets.size(); ++i) {
    if (removed[i]) continue;
    keep.push_back(dets[i]);
    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (removed[j] || dets[j].cls != dets[i].cls) continue;
      if (iou_of(dets[i], dets[j]) > p.iou) removed[j] = 1;
    }
  }
  return keep;
}

void compute_mask160(const float* coeffs, int idx, const float* protos, float* out160) {
  const int P = kProtoSize * kProtoSize;
  for (int j = 0; j < P; ++j) out160[j] = 0.f;
  for (int c = 0; c < kMaskCoeffs; ++c) {
    const float k = coeffs[c * kNumAnchors + idx];
    const float* pr = protos + static_cast<size_t>(c) * P;
    for (int j = 0; j < P; ++j) out160[j] += k * pr[j];
  }
  for (int j = 0; j < P; ++j) out160[j] = 1.f / (1.f + std::exp(-out160[j]));
}

} // namespace adas
