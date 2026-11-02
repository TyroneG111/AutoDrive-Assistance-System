#include "perception/rules/rule_engine.h"

#include "common/types.h"

#include <algorithm>
#include <cmath>

namespace adas {

const char* danger_name(DangerType t) {
  switch (t) {
    case DangerType::RedLightRunning:    return "闯红灯";
    case DangerType::WrongWay:           return "逆行";
    case DangerType::CrossingCenterLine: return "压实线";   // 已并入压实线
    case DangerType::CrossSolidLine:     return "压实线";
    case DangerType::CrossDashedLine:    return "压虚线";
    case DangerType::WrongLane:          return "未按导引箭头行驶";
    case DangerType::CollisionWarning:   return "碰撞预警";
    case DangerType::LaneKeepViolation:  return "请保持当前车道";
    case DangerType::None:               break;
  }
  return "无";
}

// 终端日志用英文名：板卡终端/串口编码不一定支持 UTF-8，中文会乱码
const char* danger_name_en(DangerType t) {
  switch (t) {
    case DangerType::RedLightRunning:    return "RedLightRunning";
    case DangerType::WrongWay:           return "WrongWay";
    case DangerType::CrossingCenterLine: return "CrossSolidLine";
    case DangerType::CrossSolidLine:     return "CrossSolidLine";
    case DangerType::CrossDashedLine:    return "CrossDashedLine";
    case DangerType::WrongLane:          return "WrongLane";
    case DangerType::CollisionWarning:   return "CollisionWarning";
    case DangerType::LaneKeepViolation:  return "LaneKeep";
    case DangerType::None:               break;
  }
  return "None";
}

int danger_rank(DangerType t) {
  switch (t) {
    case DangerType::RedLightRunning:    return 0;
    case DangerType::WrongWay:           return 1;
    case DangerType::CrossingCenterLine: return 2;
    case DangerType::CrossSolidLine:     return 3;
    case DangerType::LaneKeepViolation:  return 4;
    case DangerType::CrossDashedLine:    return 5;
    case DangerType::WrongLane:          return 6;
    case DangerType::CollisionWarning:   return 7;
    case DangerType::None:               break;
  }
  return 99;
}

// ---------------------------------------------------------------- 参数

int RuleParams::arrow_mask(int cls) const {
  // bit0=直行 bit1=左转 bit2=右转
  switch (cls) {
    case 3:  return 0x2;   // left_arrow
    case 4:  return 0x4;   // right_arrow
    case 6:  return 0x3;   // straight_left_arrow
    case 7:  return 0x5;   // straight_right_arrow
    case 8:  return 0x1;   // straight_arrow
    default: return -1;
  }
}

// ---------------------------------------------------------------- R2 灯关联

SignalObs pick_signal(const std::vector<LightObs>& lights, uint8_t behavior,
                      float ego_cx_px, float max_dx_px) {
  SignalObs out;
  int want = -1;   // 0=直 1=左 2=右
  if (behavior & kBehavLeft)          want = 1;
  else if (behavior & kBehavRight)    want = 2;
  else if (behavior & kBehavStraight) want = 0;

  // 一帧内可能同时检出多个方向、多组灯。先把候选按 (方向优先级 tier, 横向距离) 收集，
  // 再在该 tier 内做红绿仲裁。绝不能跨方向代用（旁边车道的左转红灯 ≠ 本车道直行红灯）。
  struct Cand {
    float tier;
    float dx;
    int   dir;
    int   cls;
    int   color;   // 0=红 1=绿 2=黄
    float score;
  };
  std::vector<Cand> cands;
  cands.reserve(lights.size());

  for (const auto& l : lights) {
    if (l.cls < 0 || l.cls > kNumLightClasses - 1) continue;
    const int sub = l.cls % 4;                  // 0=rnd 1=left 2=right 3=straight
    const int dir = (sub == 0) ? 3 : (sub == 1) ? 1 : (sub == 2) ? 2 : 0;

    float tier;
    if (want >= 0) {
      if (dir == want)      tier = 0.f;         // 与本车方向一致
      else if (dir == 3)    tier = 1.f;         // 圆灯（对全向同色）
      else                  continue;           // 其它方向箭头灯忽略
    } else {
      tier = (dir == 3) ? 0.f : 1.f;
    }

    const float dx = std::fabs(l.cx_px - ego_cx_px);
    // 横向门限：排除隔壁路口 / 对向车道的灯，避免多红绿灯场景误关联
    if (dx > max_dx_px) continue;

    cands.push_back({tier, dx, dir, l.cls, l.cls / 4, l.score});
  }
  if (cands.empty()) return out;

  float best_tier = 1e9f;
  for (const auto& c : cands) best_tier = std::min(best_tier, c.tier);

  const Cand* nearest = nullptr;
  float nearest_dx = 1e9f;
  const Cand* nearest_red = nullptr;
  float nearest_red_dx = 1e9f;
  for (const auto& c : cands) {
    if (c.tier > best_tier + 1e-3f) continue;
    if (c.dx < nearest_dx) { nearest_dx = c.dx; nearest = &c; }
    if (c.color == 0 && c.dx < nearest_red_dx) { nearest_red_dx = c.dx; nearest_red = &c; }
  }
  // 红绿冲突（同方向多组灯状态不一致）：若红灯分数不明显低于最近那盏，取红（保守）
  const Cand* pick = nearest;
  if (nearest_red && nearest_red != nearest && nearest &&
      nearest_red->score + 1e-6f >= nearest->score * 0.8f) {
    pick = nearest_red;
  }
  if (!pick) return out;

  out.valid = true;
  out.dir   = pick->dir;
  out.cls   = pick->cls;
  out.score = pick->score;
  out.red    = (pick->color == 0);
  out.yellow = (pick->color == 2);
  return out;
}

// ---------------------------------------------------------------- 几何

bool RuleEngine::lane_in_domain(const LaneTrack& tr, const RuleParams& p) {
  int n = 0;
  for (int i = 0; i < kGridN; ++i) {
    if (!tr.ok[i]) continue;
    const float d = kGridStart + i * kGridStep;
    if (d < 0.f || d > p.d_max) continue;
    if (std::fabs(tr.lat[i]) > p.x_max) continue;
    ++n;
  }
  return n >= p.min_grid;
}

float RuleEngine::footprint_overlap_area(const LaneTrack& tr, const RuleParams& p) {
  const float lim = p.veh_w * 0.5f + p.mark_half_w;
  const float band = 2.f * p.mark_half_w;
  float area = 0.f;
  for (float d = p.foot_near; d <= p.foot_far + 1e-4f; d += p.foot_step) {
    const float lat = tr.lateral_at(d);
    if (std::isnan(lat)) continue;
    if (std::fabs(lat) <= lim) area += p.foot_step * band;
  }
  return area;
}

// ---------------------------------------------------------------- 各规则

void RuleEngine::eval_collision(const SceneInput& s) {
  bool hit = false;
  float best_d = 1e9f, best_score = 0.f;
  for (const auto& o : s.objects) {
    if (o.forward <= 0.f) continue;
    float d_thr, x_thr;
    if (o.cls == 12 || o.cls == 13) {            // pedestrian / two_three_wheeler
      d_thr = p_.ped_d; x_thr = p_.ped_x;
    } else if (o.cls == 14 || o.cls == 15) {     // car / large_vehicle
      d_thr = p_.car_d; x_thr = p_.car_x;
    } else {
      continue;
    }
    if (o.forward > d_thr) continue;
    if (std::fabs(o.lateral) > x_thr) continue;
    hit = true;
    if (o.forward < best_d) { best_d = o.forward; best_score = o.score; }
  }
  if (hit) { m_coll_ = best_d; c_coll_ = best_score; }
  d_coll_.update(hit, s.dt, p_.t_coll, smooth_on(DangerType::CollisionWarning));
}

void RuleEngine::eval_lines(const SceneInput& s) {
  bool solid_hit = false, dash_hit = false;
  float a_solid = 0.f, a_dash = 0.f;
  int id_solid = -1, id_dash = -1;

  if (s.lanes) {
    for (const auto& tr : *s.lanes) {
      if (!lane_in_domain(tr, p_)) continue;
      const float area = footprint_overlap_area(tr, p_);
      if (area < p_.a_min) continue;

      // cls 1 = center_solid_line 已按用户要求并入普通实线，统一报“压实线”；
      // 其余按线型分实/虚。
      if (tr.cls == 1 || tr.style == LaneStyle::Solid) {
        if (area > a_solid) { a_solid = area; id_solid = tr.id; }
        solid_hit = true;
      } else {
        // Dashed 或 Unknown（未知按虚线保守处理，不触发压实线）
        if (area > a_dash) { a_dash = area; id_dash = tr.id; }
        dash_hit = true;
      }
    }
  }

  if (solid_hit)  { m_solid_ = a_solid; c_solid_ = 1.f; id_solid_ = id_solid; }
  if (dash_hit)   { m_dash_ = a_dash;   c_dash_ = 1.f;  id_dash_ = id_dash; }

  d_solid_.update(solid_hit, s.dt, p_.t_line, smooth_on(DangerType::CrossSolidLine));
  d_dash_.update(dash_hit, s.dt, p_.t_line, smooth_on(DangerType::CrossDashedLine));
}

void RuleEngine::eval_wrong_lane(const SceneInput& s) {
  const bool sm = smooth_on(DangerType::WrongLane);
  if (!s.ego.valid) { d_lane_.update(false, s.dt, p_.t_lane, sm); return; }

  // 本车期望方向（刹车 / 强制变道不参与判定）
  // behavior 是位标志，可能同时包含多个方向（如直行+左转），
  // 必须按位并集处理；用 else-if 取单一优先级会导致
  // “直行+左转”对纯直行箭头误报“未按导引箭头行驶”。
  int want = 0;
  if (s.behavior & kBehavLeft)     want |= 0x2;
  if (s.behavior & kBehavRight)    want |= 0x4;
  if (s.behavior & kBehavStraight) want |= 0x1;
  if (want == 0) { d_lane_.update(false, s.dt, p_.t_lane, sm); return; }

  // 变道过程中不判错道
  if (center_prev_valid_ && s.dt > 0.f) {
    const float v = (s.ego.center_lat - center_prev_) / s.dt;
    if (std::fabs(v) > p_.lat_speed_max) {
      d_lane_.update(false, s.dt, p_.t_lane, sm);
      return;
    }
  }

  // 本车道轴线上最近的箭头
  const float half_w = s.ego.width * 0.5f * (1.f + p_.lane_x_tol);
  int best_mask = -1;
  float best_fwd = 1e9f, best_dlat = 0.f, best_score = 0.f;
  for (const auto& a : s.arrows) {
    if (a.forward <= 0.f || a.forward > p_.d_max) continue;
    const float dlat = a.lateral - s.ego.center_lat;
    if (std::fabs(dlat) > half_w) continue;
    if (a.forward < best_fwd) {
      best_fwd = a.forward;
      best_dlat = dlat;
      best_score = a.score;
      best_mask = p_.arrow_mask(a.cls);
    }
  }
  if (best_mask < 0) { d_lane_.update(false, s.dt, p_.t_lane, sm); return; }

  const bool ok = (best_mask & want) != 0;
  if (!ok) { m_lane_ = best_dlat; c_lane_ = best_score; }
  d_lane_.update(!ok, s.dt, p_.t_lane, sm);
}

// 闯红灯：单帧判定，不依赖停止线。
//   判据 = (检出本方向红灯) ∧ (当帧几何上已位于停止位置下游)
// "已越过"由四条几何证据任一成立给出，全部是当帧量，不使用任何跨帧状态：
//   E1 停止线已在车后          stop_lines[].forward < -stop_behind_m
//   E2 斑马线整体在车后        crosswalks[].forward_max < -crosswalk_behind_m
//   E3 本车道最近的箭头在车后  arrows[].forward < -arrow_behind_m
//   E4 车头周围已无车道线      左右线最近有效网格都超过 lane_gone_m（路口内无标线）
void RuleEngine::eval_red_light(const SceneInput& s) {
  const bool sm = smooth_on(DangerType::RedLightRunning);
  const bool red = s.signal.valid && s.signal.red;
  if (!red) {
    d_red_.update(false, s.dt, p_.t_red, sm);
    return;
  }

  // 方向可信度：只检出箭头灯、但驾驶员方向未知时，无法确认这盏灯属于本车方向，
  // 不判定（否则会把隔壁车道的左转红灯当成直行红灯）。圆灯对全向有效。
  const bool dir_known = (s.signal.dir == 3) ||
                         (s.behavior & (kBehavLeft | kBehavRight | kBehavStraight)) != 0;
  if (!dir_known) {
    d_red_.update(false, s.dt, p_.t_red, sm);
    return;
  }

  bool crossed = false;
  float metric = 0.f;   // 事件"度量"列：给出最相关的那条证据距离，没有则 0

  // E1 停止线已在车后
  if (!crossed) {
    for (const auto& sl : s.stop_lines) {
      if (sl.forward < -p_.stop_behind_m) {
        crossed = true;
        metric = sl.forward;
        break;
      }
    }
  }

  // E2 斑马线整体在车后（停止线在斑马线上游，斑马线全过 = 已进入路口）
  if (!crossed) {
    for (const auto& cw : s.crosswalks) {
      if (cw.forward_max < -p_.crosswalk_behind_m) {
        crossed = true;
        metric = cw.forward_max;
        break;
      }
    }
  }

  // E3 本车道最近的箭头已在车后（箭头位于停止区上游，箭头过完就快到路口）
  if (!crossed && s.ego.valid) {
    const float half_w = s.ego.width * 0.5f * (1.f + p_.lane_x_tol);
    for (const auto& a : s.arrows) {
      if (std::fabs(a.lateral - s.ego.center_lat) > half_w) continue;
      if (a.forward < -p_.arrow_behind_m) {
        crossed = true;
        metric = a.forward;
        break;
      }
    }
  }

  // E4 车头周围已无车道线：路口内没有标线，左右线的最近有效网格都会退到较远处。
  // 要求左右两条线同时满足，避免单侧漏检造成误判。
  if (!crossed && s.lanes && s.ego.valid) {
    int gone = 0;
    for (const auto& tr : *s.lanes) {
      if (tr.id != s.ego.left_id && tr.id != s.ego.right_id) continue;
      int i0 = -1;
      for (int i = 0; i < kGridN; ++i) {
        if (tr.ok[i]) { i0 = i; break; }
      }
      if (i0 < 0) continue;
      const float d = kGridStart + static_cast<float>(i0) * kGridStep;
      if (d > p_.lane_gone_m) ++gone;
    }
    if (gone >= 2) crossed = true;
  }

  if (crossed) m_red_ = metric;
  d_red_.update(crossed, s.dt, p_.t_red, sm);
}


// ---------------------------------------------------------------- 主入口

// R8 车道保持：驾驶员要求强制变道，但本车道左右任一侧是实线 -> 提示保持当前车道。
// 只看当帧：behavior 位 + 本车道左右线型，不依赖任何跨帧状态。
void RuleEngine::eval_lane_keep(const SceneInput& s) {
  const bool force_change = (s.behavior & kBehavForceLaneChange) != 0;
  const bool sm = smooth_on(DangerType::LaneKeepViolation);
  if (!force_change || !s.ego.valid) {
    d_keep_.update(false, s.dt, p_.t_lane, sm);
    return;
  }
  const bool solid = (s.ego.left_style == LaneStyle::Solid) ||
                     (s.ego.right_style == LaneStyle::Solid);
  if (solid) m_keep_ = s.ego.center_lat;   // 度量：本车横向位置（m）
  d_keep_.update(solid, s.dt, p_.t_lane, sm);
}

void RuleEngine::reset() {
  d_red_.reset(); d_wrongway_.reset(); d_center_.reset();
  d_solid_.reset(); d_dash_.reset(); d_lane_.reset(); d_coll_.reset(); d_keep_.reset();
  center_prev_valid_ = false; center_prev_ = 0.f;
  active_.clear();
}

DangerEvent RuleEngine::update(const SceneInput& s) {
  active_.clear();

  if (!s.has_main_camera) {
    d_red_.update(false, s.dt, p_.t_red, smooth_on(DangerType::RedLightRunning));
    d_solid_.update(false, s.dt, p_.t_line, smooth_on(DangerType::CrossSolidLine));
    d_dash_.update(false, s.dt, p_.t_line, smooth_on(DangerType::CrossDashedLine));
    d_lane_.update(false, s.dt, p_.t_lane, smooth_on(DangerType::WrongLane));
    d_coll_.update(false, s.dt, p_.t_coll, smooth_on(DangerType::CollisionWarning));
    d_keep_.update(false, s.dt, p_.t_lane, smooth_on(DangerType::LaneKeepViolation));
    return DangerEvent{};
  }

  eval_collision(s);
  eval_lines(s);
  eval_wrong_lane(s);
  eval_red_light(s);
  eval_lane_keep(s);

  auto push = [&](bool on, DangerType t, float metric, float conf, int id) {
    if (!on) return;
    // 用户在设置里关掉的报警类型直接丢弃
    if (!(p_.enabled_mask & (1u << static_cast<unsigned>(t)))) return;
    DangerEvent e;
    e.type = t; e.metric = metric; e.conf = conf; e.track_id = id;
    active_.push_back(e);
  };
  push(d_red_.on,       DangerType::RedLightRunning,  m_red_, s.signal.score, -1);
  push(d_solid_.on,     DangerType::CrossSolidLine,     m_solid_, c_solid_, id_solid_);
  push(d_dash_.on,      DangerType::CrossDashedLine,    m_dash_,  c_dash_,  id_dash_);
  push(d_lane_.on,      DangerType::WrongLane,          m_lane_,  c_lane_,  -1);
  push(d_keep_.on,      DangerType::LaneKeepViolation,  m_keep_,  1.f,      -1);
  push(d_coll_.on,      DangerType::CollisionWarning,   m_coll_,  c_coll_,  -1);

  std::sort(active_.begin(), active_.end(), [](const DangerEvent& a, const DangerEvent& b) {
    return danger_rank(a.type) < danger_rank(b.type);
  });

  // R4 状态维护
  center_prev_valid_ = s.ego.valid;
  center_prev_ = s.ego.center_lat;

  return active_.empty() ? DangerEvent{} : active_.front();
}

} // namespace adas
