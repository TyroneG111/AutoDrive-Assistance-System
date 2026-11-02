#include "render/bev_scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace adas {
namespace {

// 白底配色：底色为白，所有前景元素改用深色系保证对比度
const Rgba kBg{255, 255, 255, 255};
const Rgba kGrid{198, 203, 212, 255};
const Rgba kGridMajor{158, 165, 178, 255};
const Rgba kLaneSolid{38, 42, 50, 255};
const Rgba kLaneDashed{96, 102, 114, 255};
const Rgba kEgoFill{40, 110, 200, 40};
const Rgba kObj{64, 70, 82, 255};
const Rgba kObjDim{126, 132, 144, 255};
const Rgba kDanger{214, 42, 42, 255};
const Rgba kText{74, 80, 92, 255};

DrawCmd make_line(float x0, float y0, float x1, float y1, const Rgba& c, float w) {
  DrawCmd d;
  d.kind = DrawKind::Line;
  d.stroke = c;
  d.width = w;
  d.pts = {x0, y0, x1, y1};
  return d;
}

DrawCmd make_text(float x, float y, const char* s, const Rgba& c, float size, int align) {
  DrawCmd d;
  d.kind = DrawKind::Text;
  d.stroke = c;
  d.text = s;
  d.text_size = size;
  d.align = align;
  d.pts = {x, y};
  return d;
}

const char* arrow_label(int cls) {
  switch (cls) {
    case 3: return "左转";
    case 4: return "右转";
    case 6: return "直左";
    case 7: return "直右";
    case 8: return "直行";
    default: return "箭头";
  }
}

IconId icon_of_class(int cls) {
  switch (cls) {
    case 12: return IconId::Pedestrian;
    case 13: return IconId::TwoWheeler;
    case 15: return IconId::LargeVehicle;
    case 14:
    default: return IconId::Car;
  }
}

// 取一条车道轨迹（即一个掩膜）纵向上的两个端点，BEV 只用这两点连直线。
// 逐点连线会被网格累积的碎点拉出斜齿，改用首尾端点更稳也更符合实际车道线。
bool track_ends(const LaneTrack& tr, const BevMap& mp,
                float* x0, float* y0, float* x1, float* y1) {
  int i0 = -1, i1 = -1;
  for (int i = 0; i < kGridN; ++i) {
    if (!tr.ok[i]) continue;
    if (i0 < 0) i0 = i;
    i1 = i;
  }
  if (i0 < 0 || i1 <= i0) return false;
  const float d0 = kGridStart + static_cast<float>(i0) * kGridStep;
  const float d1 = kGridStart + static_cast<float>(i1) * kGridStep;
  *x0 = mp.px(tr.lat[i0]);
  *y0 = mp.py(d0);
  *x1 = mp.px(tr.lat[i1]);
  *y1 = mp.py(d1);
  return true;
}

// 端点间的纵向跨度（米），用于“最短绘制长度”过滤
float track_span_m(const LaneTrack& tr) {
  int i0 = -1, i1 = -1;
  for (int i = 0; i < kGridN; ++i) {
    if (!tr.ok[i]) continue;
    if (i0 < 0) i0 = i;
    i1 = i;
  }
  if (i0 < 0 || i1 <= i0) return 0.f;
  return static_cast<float>(i1 - i0) * kGridStep;
}

const LaneTrack* find_track(const std::vector<LaneTrack>& v, int id) {
  if (id < 0) return nullptr;
  for (const auto& t : v) {
    if (t.id == id) return &t;
  }
  return nullptr;
}

} // namespace

void icon_size_m(IconId id, float* w_m, float* h_m) {
  float w = 1.8f, h = 4.6f;
  switch (id) {
    case IconId::EgoCar:       w = 1.8f; h = 4.6f; break;
    case IconId::Car:          w = 1.9f; h = 4.6f; break;
    case IconId::LargeVehicle: w = 2.5f; h = 6.0f; break;
    case IconId::Pedestrian:   w = 1.4f; h = 1.4f; break;   // 放大一倍（原 0.7/0.7）
    case IconId::TwoWheeler:   w = 1.8f; h = 4.0f; break;   // 放大一倍（原 0.9/2.0）
    default: break;
  }
  if (w_m) *w_m = w;
  if (h_m) *h_m = h;
}

void build_bev_scene(const BevInput& in, const BevParams& p, BevScene* out) {
  if (!out) return;
  out->cmds.clear();
  out->width = p.canvas_w;
  out->height = p.canvas_h;

  const BevMap mp(p);
  auto& cs = out->cmds;

  // 背景
  {
    DrawCmd d;
    d.kind = DrawKind::Rect;
    d.fill = kBg;
    d.stroke = kBg;
    d.pts = {0.f, 0.f, static_cast<float>(p.canvas_w), static_cast<float>(p.canvas_h)};
    cs.push_back(std::move(d));
  }

  // 网格：横向（等纵向距离）
  {
    const float x0 = mp.px(p.lat_half), x1 = mp.px(-p.lat_half);
    for (float X = 0.f; X <= p.fwd_max + 1e-3f; X += p.grid_m) {
      const float y = mp.py(X);
      if (y < 0.f || y > static_cast<float>(p.canvas_h)) continue;
      cs.push_back(make_line(x0, y, x1, y, X == 0.f ? kGridMajor : kGrid, 1.f));
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%dm", static_cast<int>(X));
      cs.push_back(make_text(x0 + 6.f, y - 4.f, buf, kText, 14.f, 0));
    }
    // 纵向
    for (float Y = -p.lat_half; Y <= p.lat_half + 1e-3f; Y += p.grid_m) {
      const float x = mp.px(Y);
      if (x < 0.f || x > static_cast<float>(p.canvas_w)) continue;
      cs.push_back(make_line(x, mp.py(p.fwd_min), x, mp.py(p.fwd_max), kGrid, 1.f));
    }
  }

  // 本车道填充（左右线各自两端点组成的四边形，与直线渲染保持一致）
  if (in.lanes && in.ego.valid) {
    const LaneTrack* lt = find_track(*in.lanes, in.ego.left_id);
    const LaneTrack* rt = find_track(*in.lanes, in.ego.right_id);
    float lx0, ly0, lx1, ly1, rx0, ry0, rx1, ry1;
    if (lt && rt && track_ends(*lt, mp, &lx0, &ly0, &lx1, &ly1) &&
        track_ends(*rt, mp, &rx0, &ry0, &rx1, &ry1)) {
      DrawCmd d;
      d.kind = DrawKind::Polygon;
      d.fill = kEgoFill;
      d.stroke = Rgba{0, 0, 0, 0};
      // 左近 -> 左远 -> 右远 -> 右近
      d.pts = {lx0, ly0, lx1, ly1, rx1, ry1, rx0, ry0};
      cs.push_back(std::move(d));
    }
  }

  // 车道线：取该掩膜的两个端点，在 BEV 上直接连直线
  if (in.lanes) {
    for (const auto& tr : *in.lanes) {
      float x0, y0, x1, y1;
      if (!track_ends(tr, mp, &x0, &y0, &x1, &y1)) continue;

      // 纵向长度过滤：太短的碎线不画（可在设置页调）
      if (p.lane_min_m > 0.f && track_span_m(tr) < p.lane_min_m) continue;

      const bool solid = (tr.style == LaneStyle::Solid) || (tr.cls == 1);
      DrawCmd d;
      // 只用两个端点，几何上就是直线；用 Polyline 是为了保留虚线样式与圆头，
      // DrawKind::Line 分支不处理 dash_on/dash_off。
      d.kind = DrawKind::Polyline;
      d.pts = {x0, y0, x1, y1};
      d.stroke = solid ? kLaneSolid : kLaneDashed;
      d.width = solid ? 5.f : 4.f;
      if (!solid) {
        d.dash_on = 2.0f * mp.ppm;
        d.dash_off = 4.0f * mp.ppm;
      }
      cs.push_back(std::move(d));
    }
  }

  // 前方目标
  for (const auto& o : in.objects) {
    if (o.forward < p.fwd_min || o.forward > p.fwd_max) continue;
    if (std::fabs(o.lateral) > p.lat_half) continue;
    const float x = mp.px(o.lateral);
    // (o.forward, o.lateral) 是包围盒底边中点投影 -> 目标“后侧接地点”，
    // 图标以车辆中心绘制，所以向前偏移半个车身长度。
    const float y_contact = mp.py(o.forward);

    const IconId id = icon_of_class(o.cls);
    float wm = 0.f, hm = 0.f;
    icon_size_m(id, &wm, &hm);
    const float y = mp.py(o.forward + 0.5f * hm);
    DrawCmd ic;
    ic.kind = DrawKind::Icon;
    ic.icon = id;
    ic.icon_w = wm * mp.ppm;
    ic.icon_h = hm * mp.ppm;
    ic.pts = {x, y};
    ic.stroke = kObj;
    ic.fill = Rgba{0, 0, 0, 0};
    ic.angle_deg = 0.f;
    cs.push_back(std::move(ic));

    // 地面接地点（车尾与地面接触点）
    DrawCmd dot;
    dot.kind = DrawKind::Circle;
    dot.radius = 3.f;
    dot.fill = kObjDim;
    dot.pts = {x, y_contact};
    cs.push_back(std::move(dot));

    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1fm", static_cast<double>(o.forward));
    cs.push_back(make_text(x, y_contact + 14.f, buf, kText, 13.f, 1));
  }

  // 本车
  {
    const float x = mp.px(0.f);
    const float y = mp.py(0.f);
    DrawCmd ic;
    ic.kind = DrawKind::Icon;
    ic.icon = IconId::EgoCar;
    ic.icon_w = p.ego_w * mp.ppm;
    ic.icon_h = p.ego_len * mp.ppm;
    ic.pts = {x, y};
    ic.stroke = (in.danger != DangerType::None) ? kDanger : Rgba{24, 96, 180, 255};
    ic.fill = Rgba{0, 0, 0, 0};
    cs.push_back(std::move(ic));
  }

  // 地面导引箭头：必须画在本车之后，否则近距离（< 半个车长）的箭头会被本车图标盖住
  for (const auto& a : in.arrows) {
    if (a.forward < p.fwd_min || a.forward > p.fwd_max) continue;
    if (std::fabs(a.lateral) > p.lat_half) continue;
    const float x = mp.px(a.lateral);
    const float y = mp.py(a.forward);

    DrawCmd dot;
    dot.kind = DrawKind::Circle;
    dot.radius = 7.f;
    dot.fill = Rgba{46, 190, 96, 235};
    dot.stroke = Rgba{16, 120, 56, 255};
    dot.pts = {x, y};
    cs.push_back(std::move(dot));

    cs.push_back(make_text(x, y - 12.f, arrow_label(a.cls), Rgba{16, 130, 58, 255}, 15.f, 1));
  }

  // 危险提示横幅
  if (in.danger != DangerType::None) {
    DrawCmd box;
    box.kind = DrawKind::RoundRect;
    box.radius = 10.f;
    box.fill = Rgba{kDanger.r, kDanger.g, kDanger.b, 210};
    box.stroke = kDanger;
    box.pts = {16.f, 16.f, static_cast<float>(p.canvas_w) - 16.f, 76.f};
    cs.push_back(std::move(box));

    const char* label = in.danger_text.empty() ? danger_name(in.danger)
                                               : in.danger_text.c_str();
    cs.push_back(make_text(0.5f * static_cast<float>(p.canvas_w), 45.f, label,
                           Rgba{255, 255, 255, 255}, 30.f, 1));
  }
}

} // namespace adas
