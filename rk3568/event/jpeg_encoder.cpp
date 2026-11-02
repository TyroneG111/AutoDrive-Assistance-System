#include "event/jpeg_encoder.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace adas {
namespace {

// ---------------- 标准表（JPEG 规范 Annex K）----------------
const uint8_t kZigZag[64] = {
    0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

const uint8_t kQuantLuma[64] = {
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};

const uint8_t kQuantChroma[64] = {
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};

const uint8_t kDcLumaBits[17] = {0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
const uint8_t kDcLumaVals[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

const uint8_t kDcChromaBits[17] = {0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
const uint8_t kDcChromaVals[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

const uint8_t kAcLumaBits[17] = {0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
const uint8_t kAcLumaVals[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51,
    0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1,
    0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18,
    0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57,
    0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92,
    0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8,
    0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2,
    0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

const uint8_t kAcChromaBits[17] = {0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};
const uint8_t kAcChromaVals[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07,
    0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09,
    0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
    0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56,
    0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
    0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
    0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2,
    0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

// ---------------- Huffman 码表 ----------------
struct HuffTable {
  uint16_t code[256] = {0};
  uint8_t  size[256] = {0};

  void build(const uint8_t bits[17], const uint8_t vals[]) {
    int k = 0;
    uint16_t c = 0;
    for (int len = 1; len <= 16; ++len) {
      for (int i = 0; i < bits[len]; ++i) {
        code[vals[k]] = c;
        size[vals[k]] = static_cast<uint8_t>(len);
        ++c;
        ++k;
      }
      c <<= 1;
    }
  }
};

struct BitWriter {
  std::string* out = nullptr;
  uint32_t buf = 0;
  int cnt = 0;

  void put(int code, int len) {
    if (len <= 0) return;
    buf = (buf << len) | (static_cast<uint32_t>(code) & ((1u << len) - 1u));
    cnt += len;
    while (cnt >= 8) {
      cnt -= 8;
      raw(static_cast<uint8_t>((buf >> cnt) & 0xFF));
    }
  }
  void raw(uint8_t b) {
    out->push_back(static_cast<char>(b));
    if (b == 0xFF) out->push_back(0);
  }
  void flush() {
    if (cnt > 0) {
      buf <<= (8 - cnt);
      raw(static_cast<uint8_t>(buf & 0xFF));
      cnt = 0;
    }
    buf = 0;
  }
};

float g_cos[8][8];
bool g_cos_ready = false;

void init_cos() {
  if (g_cos_ready) return;
  const double pi = 3.14159265358979323846;
  for (int u = 0; u < 8; ++u) {
    const double cu = (u == 0) ? std::sqrt(1.0 / 8.0) : std::sqrt(2.0 / 8.0);
    for (int x = 0; x < 8; ++x)
      g_cos[u][x] = static_cast<float>(cu * std::cos((2.0 * x + 1.0) * u * pi / 16.0));
  }
  g_cos_ready = true;
}

// 8x8 前向 DCT（分离式）
void fdct8x8(const float in[64], float out[64]) {
  float tmp[64];
  for (int y = 0; y < 8; ++y) {
    for (int u = 0; u < 8; ++u) {
      float s = 0.f;
      for (int x = 0; x < 8; ++x) s += in[y * 8 + x] * g_cos[u][x];
      tmp[y * 8 + u] = s;
    }
  }
  for (int u = 0; u < 8; ++u) {
    for (int v = 0; v < 8; ++v) {
      float s = 0.f;
      for (int y = 0; y < 8; ++y) s += tmp[y * 8 + u] * g_cos[v][y];
      out[v * 8 + u] = s;
    }
  }
}

uint8_t scale_quant(int base, int quality) {
  const int q = quality < 1 ? 1 : (quality > 100 ? 100 : quality);
  const int scale = (q < 50) ? (5000 / q) : (200 - q * 2);
  int v = (base * scale + 50) / 100;
  if (v < 1) v = 1;
  if (v > 255) v = 255;
  return static_cast<uint8_t>(v);
}

int magnitude_category(int v) {
  int a = v < 0 ? -v : v;
  int n = 0;
  while (a) { ++n; a >>= 1; }
  return n;
}

void put_marker(std::string* out, uint8_t m) {
  out->push_back(static_cast<char>(0xFF));
  out->push_back(static_cast<char>(m));
}

void put_u16(std::string* out, int v) {
  out->push_back(static_cast<char>((v >> 8) & 0xFF));
  out->push_back(static_cast<char>(v & 0xFF));
}

void encode_block(BitWriter& bw, const float blk[64], const uint8_t quant[64],
                  const HuffTable& dc, const HuffTable& ac, int& prev_dc) {
  float coeff[64];
  fdct8x8(blk, coeff);

  int q[64];
  for (int i = 0; i < 64; ++i) {
    const int zi = kZigZag[i];
    q[i] = static_cast<int>(std::lround(coeff[zi] / static_cast<float>(quant[zi])));
  }

  // DC
  const int diff = q[0] - prev_dc;
  prev_dc = q[0];
  const int cat = magnitude_category(diff);
  bw.put(dc.code[cat], dc.size[cat]);
  if (cat) {
    const int v = diff < 0 ? diff - 1 : diff;
    bw.put(v & ((1 << cat) - 1), cat);
  }

  // AC
  int run = 0;
  for (int i = 1; i < 64; ++i) {
    if (q[i] == 0) { ++run; continue; }
    while (run > 15) {
      bw.put(ac.code[0xF0], ac.size[0xF0]);
      run -= 16;
    }
    const int c = magnitude_category(q[i]);
    const int sym = (run << 4) | c;
    bw.put(ac.code[sym], ac.size[sym]);
    const int v = q[i] < 0 ? q[i] - 1 : q[i];
    bw.put(v & ((1 << c) - 1), c);
    run = 0;
  }
  if (run > 0) bw.put(ac.code[0x00], ac.size[0x00]);   // EOB
}

} // namespace

bool encode_jpeg(const uint8_t* rgb, int w, int h, int quality, std::string* out) {
  if (!rgb || !out || w <= 0 || h <= 0) return false;
  init_cos();

  static HuffTable dc_luma, ac_luma, dc_chroma, ac_chroma;
  static bool tables_ready = false;
  if (!tables_ready) {
    dc_luma.build(kDcLumaBits, kDcLumaVals);
    ac_luma.build(kAcLumaBits, kAcLumaVals);
    dc_chroma.build(kDcChromaBits, kDcChromaVals);
    ac_chroma.build(kAcChromaBits, kAcChromaVals);
    tables_ready = true;
  }

  uint8_t ql[64], qc[64];
  for (int i = 0; i < 64; ++i) {
    ql[i] = scale_quant(kQuantLuma[i], quality);
    qc[i] = scale_quant(kQuantChroma[i], quality);
  }

  out->clear();
  out->reserve(static_cast<size_t>(w) * h / 4 + 1024);

  // SOI
  put_marker(out, 0xD8);

  // APP0 (JFIF)
  put_marker(out, 0xE0);
  put_u16(out, 16);
  out->append("JFIF\0", 5);
  out->push_back(1); out->push_back(1);       // version 1.01
  out->push_back(0);                          // units: none
  put_u16(out, 1); put_u16(out, 1);           // density
  out->push_back(0); out->push_back(0);       // no thumbnail

  // DQT
  put_marker(out, 0xDB);
  put_u16(out, 2 + 65 * 2);
  out->push_back(0x00);
  for (int i = 0; i < 64; ++i) out->push_back(static_cast<char>(ql[kZigZag[i]]));
  out->push_back(0x01);
  for (int i = 0; i < 64; ++i) out->push_back(static_cast<char>(qc[kZigZag[i]]));

  // SOF0
  put_marker(out, 0xC0);
  put_u16(out, 8 + 3 * 3);
  out->push_back(8);
  put_u16(out, h);
  put_u16(out, w);
  out->push_back(3);
  out->push_back(1); out->push_back(0x11); out->push_back(0);
  out->push_back(2); out->push_back(0x11); out->push_back(1);
  out->push_back(3); out->push_back(0x11); out->push_back(1);

  // DHT
  auto put_dht = [&](uint8_t cls_id, const uint8_t bits[17], const uint8_t vals[],
                     int nvals) {
    put_marker(out, 0xC4);
    put_u16(out, 2 + 1 + 16 + nvals);
    out->push_back(static_cast<char>(cls_id));
    for (int i = 1; i <= 16; ++i) out->push_back(static_cast<char>(bits[i]));
    for (int i = 0; i < nvals; ++i) out->push_back(static_cast<char>(vals[i]));
  };
  put_dht(0x00, kDcLumaBits, kDcLumaVals, 12);
  put_dht(0x10, kAcLumaBits, kAcLumaVals, 162);
  put_dht(0x01, kDcChromaBits, kDcChromaVals, 12);
  put_dht(0x11, kAcChromaBits, kAcChromaVals, 162);

  // SOS
  put_marker(out, 0xDA);
  put_u16(out, 6 + 2 * 3);
  out->push_back(3);
  out->push_back(1); out->push_back(0x00);
  out->push_back(2); out->push_back(0x11);
  out->push_back(3); out->push_back(0x11);
  out->push_back(0); out->push_back(63); out->push_back(0);

  // 熵编码数据
  BitWriter bw;
  bw.out = out;
  int dc_y = 0, dc_cb = 0, dc_cr = 0;

  float blk[64];
  for (int by = 0; by < h; by += 8) {
    for (int bx = 0; bx < w; bx += 8) {
      float yblk[64], cbblk[64], crblk[64];
      for (int yy = 0; yy < 8; ++yy) {
        const int y = by + yy < h ? by + yy : h - 1;
        for (int xx = 0; xx < 8; ++xx) {
          const int x = bx + xx < w ? bx + xx : w - 1;
          const uint8_t* p = rgb + (static_cast<size_t>(y) * w + x) * 3;
          const float r = p[0], g = p[1], b = p[2];
          const int i = yy * 8 + xx;
          yblk[i] = 0.299f * r + 0.587f * g + 0.114f * b - 128.f;
          cbblk[i] = -0.168736f * r - 0.331264f * g + 0.5f * b;
          crblk[i] = 0.5f * r - 0.418688f * g - 0.081312f * b;
        }
      }
      std::memcpy(blk, yblk, sizeof(blk));
      encode_block(bw, blk, ql, dc_luma, ac_luma, dc_y);
      std::memcpy(blk, cbblk, sizeof(blk));
      encode_block(bw, blk, qc, dc_chroma, ac_chroma, dc_cb);
      std::memcpy(blk, crblk, sizeof(blk));
      encode_block(bw, blk, qc, dc_chroma, ac_chroma, dc_cr);
    }
  }
  bw.flush();

  put_marker(out, 0xD9);   // EOI
  return true;
}

} // namespace adas
