#pragma once
// 基线 JPEG 编码器（4:4:4，标准 Huffman 表），无外部依赖。
// 板端虽有 OpenCV，但自带编码器可让 host 端也能完整自测。
#include <cstdint>
#include <string>

namespace adas {

// rgb: w*h*3 字节；quality: 1..100
bool encode_jpeg(const uint8_t* rgb, int w, int h, int quality, std::string* out);

} // namespace adas
