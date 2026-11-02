#pragma once
#include "perception/rules/rule_types.h"

namespace adas {

// ---------- 规则参数（默认值见 docs/危险驾驶行为识别.md 第 3 章）----------
struct RuleParams {
  // R1 车道线有效域
  float d_max = 25.0f;      // 纵向有效距离 m
  float x_max = 6.0f;       // 横向有效范围 m
  int   min_grid = 2;       // 最少有效网格数

  // R5 压线
  float veh_w = 1.8f;       // 本车宽 m
  float mark_half_w = 0.15f;// 标记带半宽 m
  float a_min = 0.30f;      // 判定面积 m²
  float foot_near = 0.8f;   // 足迹纵向窗（近端，米）
  float foot_far = 2.6f;    // 足迹纵向窗（远端，米）
  float foot_step = 0.2f;   // 面积积分步长 m

  // R4 错道
  float lane_x_tol = 0.6f;  // 箭头落入本车道的横向容差（×车道宽/2）
  float lat_speed_max = 0.3f; // 变道中判定阈值 m/s

  // R3 闯红灯：单帧“已越过停止位置”判据（米），四条证据任一成立即算
  float stop_behind_m = 0.2f;       // E1 停止线已在车后
  float crosswalk_behind_m = 0.5f;  // E2 斑马线整体在车后
  float arrow_behind_m = 1.0f;      // E3 本车道箭头已在车后
  float lane_gone_m = 2.5f;         // E4 车头周围无线（左右线最近有效网格均超过此值）

  // 时间去抖（秒）
  float t_red = 0.3f;
  float t_line = 0.3f;
  float t_lane = 0.3f;
  float t_coll = 0.3f;

  // R7 碰撞预警阈值
  // 原来 8m/6m 太容易误报（正前方同一车道内的跟车就会被判为碰撞），
  // 改为物体接地点进入本车 1.5m 内才预警。
  float ped_d = 1.5f, ped_x = 2.5f;   // pedestrian / two_three_wheeler
  float car_d = 1.5f, car_x = 2.0f;   // car / large_vehicle

  // 箭头类别 -> 允许方向掩码（bit0=直行 bit1=左转 bit2=右转）
  int arrow_mask(int cls) const;

  // 报警开关：bit(1<<DangerType) 为 1 才允许该类报警（None 不参与）
  // 默认 0x1FE = bit1..bit8 全开。逆行（bit2）实际不会触发。
  uint32_t enabled_mask = 0x1FEu;

  // 多帧平滑（去抖）开关：bit(1<<DangerType) 为 1 表示该类报警需要
  // 连续满足 t_* 秒才触发；为 0 表示当帧满足即触发（单帧判定）。
  // 默认 0 = 全部单帧，保持与“取消多帧判定”后的行为一致。
  uint32_t smooth_mask = 0u;
};

// ---------- R2 信号灯—方向关联 ----------
// 从原始灯检测中选出与驾驶员行为一致、且最靠近本车道投影位置的一组灯。
//   lights      : 类别 0..11 的灯检测
//   behavior    : DriverBehavior one-hot
//   ego_cx_px   : 本车道在图像中的水平位置（缺省用图像中心）
//   max_dx_px   : 横向门限（像素），超过则视为隔壁路口/对向车道的灯而丢弃
// 返回 valid=false 表示无可用灯。
SignalObs pick_signal(const std::vector<LightObs>& lights, uint8_t behavior,
                      float ego_cx_px, float max_dx_px = 1e9f);

// ---------- 时间去抖 ----------
// smooth = false：当帧条件成立就立即判定（单帧，默认）。
// smooth = true ：条件需连续满足 need 秒才判定（多帧平滑，逐项可开）。
// 条件一旦中断立即清零并撤销，保证平滑只压制“瞬时抖动”，不延长误报。
struct Debounce {
  float t = 0.f;
  bool  on = false;
  bool update(bool cond, float dt, float need, bool smooth) {
    if (!cond) {
      t = 0.f;
      on = false;
      return on;
    }
    if (!smooth) {   // 单帧：立即生效
      t = need;
      on = true;
      return on;
    }
    if (dt < 0.f) dt = 0.f;
    t += dt;
    if (t >= need) on = true;
    return on;
  }
  void reset() { t = 0.f; on = false; }
};

// ---------- 规则引擎 ----------
class RuleEngine {
public:
  void reset();

  // 每帧调用一次；返回本帧生效的最高优先级危险（type=None 表示无）
  DangerEvent update(const SceneInput& s);

  // 最近一帧所有已生效规则（按优先级降序），便于界面/日志展示
  const std::vector<DangerEvent>& active() const { return active_; }

  RuleParams& params() { return p_; }
  const RuleParams& params() const { return p_; }

  // 单项规则调试接口
  static float footprint_overlap_area(const LaneTrack& tr, const RuleParams& p);
  static bool  lane_in_domain(const LaneTrack& tr, const RuleParams& p);

private:
  // 该类报警是否启用多帧平滑
  bool smooth_on(DangerType t) const {
    return (p_.smooth_mask & (1u << static_cast<unsigned>(t))) != 0u;
  }

  void eval_collision(const SceneInput& s);
  void eval_lines(const SceneInput& s);
  void eval_wrong_lane(const SceneInput& s);
  void eval_red_light(const SceneInput& s);
  void eval_lane_keep(const SceneInput& s);

  RuleParams p_;
  Debounce  d_red_, d_wrongway_, d_center_, d_solid_, d_dash_, d_lane_, d_coll_, d_keep_;

  float m_keep_ = 0.f;

  // 事件细节缓存（去抖生效时输出）
  float m_coll_ = 0.f,    c_coll_ = 0.f;
  float m_dash_ = 0.f,    c_dash_ = 0.f;
  float m_solid_ = 0.f,   c_solid_ = 0.f;
  int   id_solid_ = -1,   id_center_ = -1, id_dash_ = -1;
  float m_lane_ = 0.f,    c_lane_ = 0.f;
  float m_red_ = 0.f;

  // R3 闯红灯已改为单帧判定，不再保留任何跨帧状态

  // R4 状态：上次车道中心（估计横向速度）
  bool  center_prev_valid_ = false;
  float center_prev_ = 0.f;

  std::vector<DangerEvent> active_;
};

} // namespace adas
