#pragma once
// 危险驾驶行为规则的数据结构（见 docs/危险驾驶行为识别.md 第 2 章）
#include "perception/lane_style.h"
#include "perception/lane_tracker.h"

#include <cstdint>
#include <vector>

namespace adas {

// 危险行为种类（顺序即优先级，见 danger_rank）
enum class DangerType {
  None = 0,
  RedLightRunning,      // R3 闯红灯
  WrongWay,             // R6 逆行
  CrossingCenterLine,   // R6 越中心实线（已并入压实线）
  CrossSolidLine,       // R5 压实线
  CrossDashedLine,      // R5 压虚线
  WrongLane,            // R4 错道
  CollisionWarning,     // R7 碰撞预警
  LaneKeepViolation,    // R8 车道保持：任一侧是实线而驾驶员强制变道
};

const char* danger_name(DangerType t);
// 终端日志用英文名（板卡终端/串口不支持 UTF-8 时会出乱码）
const char* danger_name_en(DangerType t);

// 优先级：数值越小越优先
// 闯红灯 > 逆行/越线 > 压线(实) > 压线(虚) > 错道 > 碰撞预警
int danger_rank(DangerType t);

// ---------- 场景输入 ----------

// 地面箭头（BEV，米）。cls 同分割类别：3/4/6/7/8
struct ArrowObs {
  int   cls = 8;
  float forward = 0.f;
  float lateral = 0.f;
  float score = 0.f;
};

// 停止线（BEV 纵向坐标，米；横向应覆盖本车道）
struct StopLineObs {
  float forward = 0.f;
  float score = 0.f;
};

// 信号灯原始检测（像素中心 x，类别 0..11）
struct LightObs {
  int   cls = 0;
  float cx_px = 0.f;
  float score = 0.f;
};

// 与本车运动方向关联后的信号灯状态
struct SignalObs {
  bool  valid = false;
  bool  red = false;
  bool  yellow = false;
  int   dir = 3;      // 0=直行 1=左转 2=右转 3=圆灯(全向)
  int   cls = -1;
  float score = 0.f;
};

// 人行横道（斑马线）在 BEV 下的包围盒，用于判断行人是否站在斑马线上
struct CrosswalkObs {
  float forward_min = 0.f;
  float forward_max = 0.f;
  float lat_min = 0.f;
  float lat_max = 0.f;
};

// 前方目标（检测框底边中点投影到 BEV）
struct ObjectObs {
  int   cls = 14;
  float forward = 0.f;
  float lateral = 0.f;
  float score = 0.f;
};

// 一帧的完整输入
struct SceneInput {
  float   dt = 1.f / 30.f;
  uint8_t behavior = 0;        // DriverBehavior one-hot
  uint8_t image_mode = 0;
  bool    has_main_camera = true;

  EgoLane ego;                                     // 本车道
  const std::vector<LaneTrack>* lanes = nullptr;   // 全部跟踪线（R5/R6）

  std::vector<ArrowObs>    arrows;
  std::vector<StopLineObs> stop_lines;
  std::vector<CrosswalkObs> crosswalks;
  std::vector<ObjectObs>   objects;
  SignalObs                signal;
};

// 规则命中结果
struct DangerEvent {
  DangerType type = DangerType::None;
  float metric = 0.f;    // 度量：压线面积 m² / 目标距离 m / 车道横向偏差 m
  float conf = 0.f;      // 置信度（触发帧的得分）
  int   track_id = -1;   // 相关车道线 track id
};

} // namespace adas
