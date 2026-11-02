#include "vision/preprocess.h"

#include "common/types.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace adas {
namespace {

inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

void resize_bilinear_rgb(const uint8_t* src, int sw, int sh,
                         uint8_t* dst, int dw, int dh, int dst_stride_px) {
  const float fx = static_cast<float>(sw) / dw;
  const float fy = static_cast<float>(sh) / dh;
  for (int y = 0; y < dh; ++y) {
    float sy = (y + 0.5f) * fy - 0.5f;
    if (sy < 0.f) sy = 0.f;
    int y0 = static_cast<int>(sy);
    if (y0 > sh - 1) y0 = sh - 1;
    const int y1 = std::min(y0 + 1, sh - 1);
    const float wy = sy - static_cast<float>(y0);
    for (int x = 0; x < dw; ++x) {
      float sx = (x + 0.5f) * fx - 0.5f;
      if (sx < 0.f) sx = 0.f;
      int x0 = static_cast<int>(sx);
      if (x0 > sw - 1) x0 = sw - 1;
      const int x1 = std::min(x0 + 1, sw - 1);
      const float wx = sx - static_cast<float>(x0);
      uint8_t* o = dst + (static_cast<size_t>(y) * dst_stride_px + x) * 3;
      for (int c = 0; c < 3; ++c) {
        const int sc = c;
        const float p00 = src[(static_cast<size_t>(y0) * sw + x0) * 3 + sc];
        const float p01 = src[(static_cast<size_t>(y0) * sw + x1) * 3 + sc];
        const float p10 = src[(static_cast<size_t>(y1) * sw + x0) * 3 + sc];
        const float p11 = src[(static_cast<size_t>(y1) * sw + x1) * 3 + sc];
        const float v = (p00 * (1.f - wx) + p01 * wx) * (1.f - wy) +
                        (p10 * (1.f - wx) + p11 * wx) * wy;
        o[c] = static_cast<uint8_t>(clampf(v + 0.5f, 0.f, 255.f));
      }
    }
  }
}

} // namespace

bool preprocess(const uint8_t* rgb, int w, int h, uint8_t* out_hwc, PreprocessMeta* meta) {
  if (!rgb || w <= 0 || h <= 0) return false;

  static thread_local std::vector<uint8_t> canvas;

  const int S = kModelSize;
  const float scale = std::min(static_cast<float>(S) / w, static_cast<float>(S) / h);
  const int nw = static_cast<int>(std::lround(w * scale));
  const int nh = static_cast<int>(std::lround(h * scale));
  const int pad_x = static_cast<int>(std::lround((S - nw) / 2.0 - 0.1));
  const int pad_y = static_cast<int>(std::lround((S - nh) / 2.0 - 0.1));

  canvas.assign(static_cast<size_t>(S) * S * 3, 114);
  resize_bilinear_rgb(rgb, w, h,
                      canvas.data() + (static_cast<size_t>(pad_y) * S + pad_x) * 3,
                      nw, nh, S);

  // canvas 本身就是 HWC 排列，直接整块拷贝给 RKNN（NHWC，免一次逐像素转置）
  std::memcpy(out_hwc, canvas.data(), static_cast<size_t>(S) * S * 3);

  if (meta) {
    meta->scale = scale;
    meta->pad_x = pad_x;
    meta->pad_y = pad_y;
    meta->src_w = w;
    meta->src_h = h;
  }
  return true;
}

} // namespace adas
