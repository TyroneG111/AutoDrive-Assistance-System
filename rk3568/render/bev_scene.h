#pragma once
// 2D 鸟瞰图（BEV）绘制指令层：与 GUI 无关，便于离屏渲染与单元验证。
#include "perception/lane_tracker.h"
#include "perception/rules/rule_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace adas {

enum class DrawKind {
  Line,       // pts = 2 点
  Polyline,   // pts = N 点
  Rect,       // pts = 2 点（左上+右下）
  RoundRect,  // pts = 2 点 + radius
  Circle,     // pts = 1 点（圆心）+ radius
  Polygon,    // pts = N 点（填充）
  Text,       // pts = 1 点（锚点）+ text
  Icon,       // pts = 1 点（中心）+ icon 尺寸/角度
};

enum class IconId { EgoCar = 0, Car, LargeVehicle, Pedestrian, TwoWheeler, Count };

struct Rgba {
  uint8_t r = 0, g = 0, b = 0, a = 255;
};

struct DrawCmd {
  DrawKind kind = DrawKind::Line;
  Rgba stroke{255, 255, 255, 255};
  Rgba fill{0, 0, 0, 0};
  float width = 1.f;
  std::vector<float> pts;        // x,y 交替（画布像素）
  float radius = 0.f;            // RoundRect 圆角 / Circle 半径
  float dash_on = 0.f;           // >0 表示虚线（像素）
  float dash_off = 0.f;
  IconId icon = IconId::Car;
  float icon_w = 0.f, icon_h = 0.f;
  float angle_deg = 0.f;
  std::string text;
  float text_size = 14.f;
  int align = 1;                 // 0=左 1=中 2=右
};

// ---------- 画布参数 ----------
struct BevParams {
  int   canvas_w = 960;
  int   canvas_h = 1080;
  float fwd_min = -6.0f;    // 车后可见范围 m
  float fwd_max = 30.0f;    // 车前可见范围 m
  float lat_half = 9.0f;    // 横向 ±m
  float grid_m = 5.0f;      // 网格间距 m
  float ego_len = 4.6f;     // 本车长 m
  float ego_w = 1.8f;       // 本车宽 m
  // 车道线在 BEV 上绘制的最小纵向长度（米）。<=0 表示不限制。
  // 分段检出的碎线很短，太小会满屏小短线；太大会把真线也滤掉。
  float lane_min_m = 0.0f;
};

// 画布坐标换算：forward X 向上，lateral Y 向左
struct BevMap {
  float ppm = 1.f;   // 像素/米
  float cx = 0.f;    // 本车中心 x（像素）
  float y0 = 0.f;    // X=0 处的 y（像素）

  BevMap() = default;
  explicit BevMap(const BevParams& p) {
    const float span_m = p.fwd_max - p.fwd_min;
    const float ppm_y = (span_m > 1e-3f) ? static_cast<float>(p.canvas_h) / span_m : 1.f;
    const float ppm_x = (p.lat_half > 1e-3f)
                            ? static_cast<float>(p.canvas_w) / (2.f * p.lat_half)
                            : 1.f;
    ppm = ppm_x < ppm_y ? ppm_x : ppm_y;
    cx = 0.5f * static_cast<float>(p.canvas_w);
    y0 = static_cast<float>(p.canvas_h) - (-p.fwd_min) * ppm;
  }
  float px(float lateral_m) const { return cx - lateral_m * ppm; }
  float py(float forward_m) const { return y0 - forward_m * ppm; }
  float m(float pixels) const { return pixels / (ppm > 1e-6f ? ppm : 1.f); }
};

// ---------- 输入 ----------
struct BevInput {
  const std::vector<LaneTrack>* lanes = nullptr;
  EgoLane ego;
  std::vector<ObjectObs> objects;
  std::vector<ArrowObs> arrows;     // 地面导引箭头（BEV 米坐标）
  DangerType danger = DangerType::None;
  std::string danger_text;
  bool has_ego = false;     // 是否画本车图标（无主摄时也画）
};

struct BevScene {
  std::vector<DrawCmd> cmds;
  int width = 0;
  int height = 0;
};

// 生成一帧 BEV 绘制指令
void build_bev_scene(const BevInput& in, const BevParams& p, BevScene* out);

// 图标在画布上的显示尺寸（米 -> 像素由调用方换算）
void icon_size_m(IconId id, float* w_m, float* h_m);

} // namespace adas
