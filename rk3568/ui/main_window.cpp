#include "ui/main_window.h"

#include "ui/bev_widget.h"
#include "ui/video_widget.h"

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSpinBox>
#include <QStringList>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cstring>

namespace adas {
namespace {

// 设置持久化文件（退出再进入不会重置）
const char* kSettingsFile = "/root/adas/adas_gui.ini";

// 统一次级按钮样式：白灰配色，和顶部“设置”按钮观感一致。
// 只有“退出”保留红色警示色，不参与此样式。
QString gray_button_qss() {
  return QStringLiteral(
      "QPushButton{background:#e8ebef;color:#20242a;border:1px solid #b6bcc4;"
      "border-radius:6px;padding:2px 10px;}"
      "QPushButton:hover{background:#f4f6f8;}"
      "QPushButton:pressed{background:#d2d7dd;}"
      "QPushButton:checked{background:#ccd2da;border:1px solid #97a0aa;}");
}

QString fmt_ts(int64_t ms) {
  const QDateTime dt = QDateTime::fromMSecsSinceEpoch(ms);
  return dt.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
}

// 找 2D图形/ 图标目录：可执行文件旁 → 上一级（部署时图标在 /root/adas/2D图形）
// → 当前工作目录。以前只试了 exe 同级，导致 build/ 下永远找不到图标而退回圆角矩形。
QString find_icon_dir() {
  const QString app = QCoreApplication::applicationDirPath();
  const QStringList cands = {app + QStringLiteral("/2D图形"),
                             app + QStringLiteral("/../2D图形"),
                             QDir::currentPath() + QStringLiteral("/2D图形")};
  for (const QString& c : cands) {
    QDir d(c);
    if (!d.exists()) continue;
    const QStringList svgs = d.entryList({QStringLiteral("*.svg")}, QDir::Files);
    if (!svgs.isEmpty()) return d.absolutePath();
  }
  return QString();
}

} // namespace

MainWindow::MainWindow(const PipelineConfig& cfg, QWidget* parent)
    : QMainWindow(parent), cfg_(cfg) {
  setWindowTitle(QStringLiteral("智能辅助驾驶 — RK3568 ADAS"));
  resize(1920, 1080);
  loadSettings();      // 先恢复上次保存的设置，控件初始化时才用得上
  buildUi();
  buildSettingsPanel();
  buildNavPanel();

  timer_ = new QTimer(this);
  timer_->setInterval(33);
  connect(timer_, &QTimer::timeout, this, [this]() { onTick(); });
  timer_->start();

  // 参数就绪后自动开始（有 --pcap 或 --udp 时）
  pipeline_.update_live(cfg_);
  setRunning(true);
}

MainWindow::~MainWindow() { pipeline_.stop(); }

void MainWindow::setIconDir(const QString& dir) {
  if (bev_) bev_->setIconDir(dir);
}

void MainWindow::buildUi() {
  auto* central = new QWidget(this);
  auto* root = new QVBoxLayout(central);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  // ---- 顶部状态条 ----
  auto* top = new QWidget(central);
  top->setFixedHeight(52);
  auto* topLay = new QHBoxLayout(top);
  topLay->setContentsMargins(4, 0, 4, 0);
  topLay->setSpacing(12);

  auto* title = new QLabel(QStringLiteral("智能辅助驾驶系统"), top);
  QFont tf = title->font();
  tf.setPixelSize(22);
  tf.setBold(true);
  title->setFont(tf);
  topLay->addWidget(title);

  statusRun_ = new QLabel(QStringLiteral("● 已停止"), top);
  statusFps_ = new QLabel(QStringLiteral("— fps"), top);
  statusBehav_ = new QLabel(QStringLiteral("行为 —"), top);
  statusDanger_ = new QLabel(QStringLiteral("无告警"), top);
  statusAdvice_ = new QLabel(QStringLiteral(""), top);
  statusEgo_ = new QLabel(QStringLiteral("本车道 —"), top);
  statusEvent_ = new QLabel(QStringLiteral("事件 0"), top);
  for (QLabel* l : {statusRun_, statusFps_, statusBehav_, statusDanger_, statusAdvice_, statusEgo_,
                    statusEvent_}) {
    QFont f = l->font();
    f.setPixelSize(16);
    l->setFont(f);
    // 不要用 Ignored：会把标签压成 0 宽，帧率/推理时间就看不见了
    l->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    topLay->addWidget(l);
  }
  topLay->addStretch(1);

  // 显示模式按钮：点击在 识别结果 → 原图 → 双目融合 之间循环
  btnView_ = new QPushButton(QStringLiteral("识别结果"), top);
  btnStart_ = new QPushButton(QStringLiteral("开始"), top);
  btnNav_ = new QPushButton(QStringLiteral("导航"), top);
  btnSettings_ = new QPushButton(QStringLiteral("设置"), top);
  btnQuit_ = new QPushButton(QStringLiteral("退出"), top);
  for (QPushButton* b : {btnView_, btnStart_, btnNav_, btnSettings_, btnQuit_}) {
    b->setMinimumHeight(36);
    b->setMinimumWidth(90);
    QFont f = b->font();
    f.setPixelSize(16);
    b->setFont(f);
    topLay->addWidget(b);
  }
  // 顶部除“退出”外统一白灰配色（与“设置”按钮一致）
  for (QPushButton* b : {btnView_, btnStart_, btnNav_, btnSettings_})
    b->setStyleSheet(gray_button_qss());

  // 导航按钮：弹出模拟导航悬浮窗
  connect(btnNav_, &QPushButton::clicked, this, [this]() { toggleNavPanel(); });

  // 退出按钮：close() 会走 closeEvent -> 保存设置后退出
  btnQuit_->setStyleSheet(QStringLiteral(
      "QPushButton{background:#c0392b;color:#fff;border-radius:4px;}"
      "QPushButton:pressed{background:#962d22;}"));
  connect(btnQuit_, &QPushButton::clicked, this, [this]() { close(); });

  connect(btnSettings_, &QPushButton::clicked, this, [this]() { toggleSettingsPanel(); });

  connect(btnView_, &QPushButton::clicked, this, [this]() { cycleViewMode(); });
  connect(btnStart_, &QPushButton::clicked, this, [this]() { setRunning(!running_); });

  root->addWidget(top);

  // ---- 中部：视频 + BEV ----
  auto* mid = new QWidget(central);
  auto* midLay = new QHBoxLayout(mid);
  midLay->setContentsMargins(0, 0, 0, 0);
  midLay->setSpacing(8);

  video_ = new VideoWidget(mid);
  bev_ = new BevWidget(mid);
  const QString iconDir = find_icon_dir();
  if (!iconDir.isEmpty()) bev_->setIconDir(iconDir);
  midLay->addWidget(video_, 3);
  midLay->addWidget(bev_, 2);
  root->addWidget(mid, 1);

  // ---- 底部：事件列表 ----
  events_ = new QTableWidget(0, 5, central);
  events_->setHorizontalHeaderLabels({QStringLiteral("时间"), QStringLiteral("行为"),
                                      QStringLiteral("度量"), QStringLiteral("置信度"),
                                      QStringLiteral("上传")});
  events_->verticalHeader()->setVisible(false);
  events_->horizontalHeader()->setStretchLastSection(true);
  events_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  events_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  events_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  events_->setSelectionBehavior(QAbstractItemView::SelectRows);
  events_->setFixedHeight(230);
  QFont ef = events_->font();
  ef.setPixelSize(15);
  events_->setFont(ef);
  root->addWidget(events_);

  setCentralWidget(central);
}

void MainWindow::buildSettingsPanel() {
  // 关键：不用 QDockWidget。dock 参与 QMainWindow 布局，打开时会挤压
  // 中央区域，关闭后布局也不一定复原（用户反馈的“关闭后未恢复”）。
  // 这里做成覆盖式面板：手动 setGeometry，完全不参与主布局。
  settingsPanel_ = new QWidget(this);
  settingsPanel_->setObjectName(QStringLiteral("settingsPanel"));
  settingsPanel_->setAttribute(Qt::WA_StyledBackground, true);
  // 白色包边 + 白色底：与整体浅色风格一致
  // （面板内部是白色滚动区，若外框仍用深色会留下一条深灰边，显得割裂）
  settingsPanel_->setStyleSheet(QStringLiteral(
      "QWidget#settingsPanel{background:#ffffff;border:2px solid #ffffff;}"));

  auto* outer = new QVBoxLayout(settingsPanel_);
  outer->setContentsMargins(10, 8, 10, 10);
  outer->setSpacing(8);

  auto* titleRow = new QHBoxLayout();
  auto* titleLbl = new QLabel(QStringLiteral("设置"), settingsPanel_);
  {
    QFont tf = titleLbl->font();
    tf.setPixelSize(20);
    tf.setBold(true);
    titleLbl->setFont(tf);
  }
  titleRow->addWidget(titleLbl);
  titleRow->addStretch(1);
  auto* btnHide = new QPushButton(QStringLiteral("收起"), settingsPanel_);
  btnHide->setMinimumHeight(34);
  btnHide->setMinimumWidth(90);
  btnHide->setStyleSheet(gray_button_qss());
  titleRow->addWidget(btnHide);
  outer->addLayout(titleRow);

  auto* body = new QWidget();   // 无 parent，交给 QScrollArea 接管
  auto* lay = new QVBoxLayout(body);
  lay->setContentsMargins(4, 4, 4, 4);
  lay->setSpacing(10);

  auto make_group = [&](const QString& name) {
    auto* g = new QGroupBox(name, body);
    auto* f = new QFormLayout(g);
    f->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    // 长标签自动换行 + 输入框占满剩余宽度，避免内容被压扁/裁切
    f->setRowWrapPolicy(QFormLayout::WrapLongRows);
    f->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    f->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
    g->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    lay->addWidget(g);
    return f;
  };

  // 数据源
  {
    auto* f = make_group(QStringLiteral("数据源（需重启流水线）"));
    spUdpPort_ = new QSpinBox(body);
    spUdpPort_->setRange(1, 65535);
    spUdpPort_->setValue(cfg_.udp_port);
    edBind_ = new QLineEdit(QString::fromStdString(cfg_.bind_ip), body);
    f->addRow(QStringLiteral("UDP 端口"), spUdpPort_);
    f->addRow(QStringLiteral("绑定 IP"), edBind_);
    spMainCam_ = new QSpinBox(body);
    spMainCam_->setRange(1, 4);
    spMainCam_->setValue(cfg_.main_cam < 1 ? 1 : (cfg_.main_cam > 4 ? 4 : cfg_.main_cam));
    f->addRow(QStringLiteral("主摄（1-4）"), spMainCam_);
    auto* camTip = new QLabel(
        QStringLiteral("四路拼接(mode=0x01)时：1左上/2右上/3左下/4右下；\n"
                       "单路模式(mode=camN)整帧即该路，不受此设置影响"),
        body);
    camTip->setWordWrap(true);
    f->addRow(camTip);
    auto* hint = new QLabel(QStringLiteral("数据源：%1")
                                .arg(cfg_.pcap.empty()
                                         ? QStringLiteral("UDP")
                                         : QString::fromStdString(cfg_.pcap)),
                            body);
    f->addRow(hint);
  }

  // 识别阈值
  {
    auto* f = make_group(QStringLiteral("识别阈值（实时生效）"));
    auto mk = [&](double v) {
      auto* s = new QDoubleSpinBox(body);
      s->setRange(0.05, 0.95);
      s->setSingleStep(0.05);
      s->setDecimals(2);
      s->setValue(v);
      return s;
    };
    spConfLight_ = mk(cfg_.conf_light);
    spConfDef_ = mk(cfg_.conf_def);
    spConfSeg_ = mk(cfg_.conf_seg);
    spIou_ = mk(cfg_.iou);
    spIouSeg_ = mk(cfg_.iou_seg);
    f->addRow(QStringLiteral("信号灯"), spConfLight_);
    f->addRow(QStringLiteral("其它目标"), spConfDef_);
    f->addRow(QStringLiteral("分割"), spConfSeg_);
    f->addRow(QStringLiteral("NMS IoU（检测）"), spIou_);
    f->addRow(QStringLiteral("NMS IoU（分割）"), spIouSeg_);
  }

  // BEV
  {
    auto* f = make_group(QStringLiteral("鸟瞰图（实时生效）"));
    spFwdMax_ = new QDoubleSpinBox(body);
    spFwdMax_->setRange(10.0, 60.0);
    spFwdMax_->setSingleStep(5.0);
    spFwdMax_->setValue(cfg_.bev_fwd_max);
    spLatHalf_ = new QDoubleSpinBox(body);
    spLatHalf_->setRange(4.0, 20.0);
    spLatHalf_->setSingleStep(1.0);
    spLatHalf_->setValue(cfg_.bev_lat_half);
    f->addRow(QStringLiteral("前向范围 m"), spFwdMax_);
    f->addRow(QStringLiteral("横向半宽 m"), spLatHalf_);
    spLaneMin_ = new QDoubleSpinBox(body);
    spLaneMin_->setRange(0.0, 20.0);
    spLaneMin_->setSingleStep(0.5);
    spLaneMin_->setDecimals(1);
    spLaneMin_->setValue(cfg_.bev_lane_min_m);
    spLaneMin_->setMinimumHeight(34);
    f->addRow(QStringLiteral("车道线最短 m"), spLaneMin_);
    auto* laneTip = new QLabel(QStringLiteral("BEV 上长度不足的碎线不绘制；0 = 不限制"), body);
    laneTip->setWordWrap(true);
    f->addRow(laneTip);
    spInferEvery_ = new QSpinBox(body);
    spInferEvery_->setRange(1, 30);
    spInferEvery_->setValue(cfg_.infer_every);
    f->addRow(QStringLiteral("每 N 帧推理"), spInferEvery_);
  }

  // 相机几何（IPM / 鸟瞰投影）
  {
    auto* f = make_group(QStringLiteral("相机内参与外参（实时生效）"));
    auto mkd = [&](double v, double lo, double hi, double step, int dec) {
      auto* s = new QDoubleSpinBox(body);
      s->setRange(lo, hi);
      s->setSingleStep(step);
      s->setDecimals(dec);
      s->setValue(v);
      return s;
    };
    spCamFx_ = mkd(cfg_.cam_fx, 50.0, 5000.0, 1.0, 1);
    spCamFy_ = mkd(cfg_.cam_fy, 50.0, 5000.0, 1.0, 1);
    spCamCx_ = mkd(cfg_.cam_cx, 0.0, 4000.0, 1.0, 1);
    spCamCy_ = mkd(cfg_.cam_cy, 0.0, 4000.0, 1.0, 1);
    spCamHeight_ = mkd(cfg_.cam_height, 0.1, 10.0, 0.05, 2);
    spCamToFront_ = mkd(cfg_.cam_to_front, 0.0, 10.0, 0.1, 2);
    spCamPitch_ = mkd(cfg_.cam_pitch_deg, -30.0, 30.0, 0.5, 1);
    spCamOffsetY_ = mkd(cfg_.cam_offset_y, -2.0, 2.0, 0.05, 2);
    spCamYaw_ = mkd(cfg_.cam_yaw_deg, -20.0, 20.0, 0.1, 2);
    spCamK1_ = mkd(cfg_.cam_k1, -1.0, 1.0, 0.01, 4);
    spCamK2_ = mkd(cfg_.cam_k2, -1.0, 1.0, 0.005, 4);
    f->addRow(QStringLiteral("焦距 fx (px)"), spCamFx_);
    f->addRow(QStringLiteral("焦距 fy (px)"), spCamFy_);
    f->addRow(QStringLiteral("光心 cx (px)"), spCamCx_);
    f->addRow(QStringLiteral("光心 cy (px)"), spCamCy_);
    f->addRow(QStringLiteral("相机高度 (m)"), spCamHeight_);
    f->addRow(QStringLiteral("光心到车头 (m)"), spCamToFront_);
    f->addRow(QStringLiteral("俯仰角 (°)"), spCamPitch_);
    f->addRow(QStringLiteral("横向偏移 (m)"), spCamOffsetY_);
    f->addRow(QStringLiteral("偏航角 (°)"), spCamYaw_);
    f->addRow(QStringLiteral("畸变 k1"), spCamK1_);
    f->addRow(QStringLiteral("畸变 k2"), spCamK2_);
    auto* tip = new QLabel(QStringLiteral("内参以 1280x720 为基准；\n四路 640x360 自动按 0.5 缩放\n俯仰角正值 = 镜头低头\n横向偏移正值 = 相机在车辆中心右侧\n偏航角正值 = 光轴偏向车辆右侧；用于修正相机与车头方向不平行\n畸变矫正径向模型（广角桶形 k1 取负值，如 -0.30），\n0 = 不矫正；切向畸变不建模"), body);
    tip->setWordWrap(true);
    f->addRow(tip);
  }

  // 事件与上传
  {
    auto* f = make_group(QStringLiteral("事件记录与上传"));
    edEventsDir_ = new QLineEdit(QString::fromStdString(cfg_.events_dir), body);
    spMaxMb_ = new QSpinBox(body);
    spMaxMb_->setRange(0, 20000);
    spMaxMb_->setValue(static_cast<int>(cfg_.event_max_mb));
    edUpload_ = new QLineEdit(QString::fromStdString(cfg_.upload_url), body);
    f->addRow(QStringLiteral("目录"), edEventsDir_);
    f->addRow(QStringLiteral("容量 MB"), spMaxMb_);
    f->addRow(QStringLiteral("上传 URL"), edUpload_);
  }

  // ---- 容错（丢包容忍）与推理策略 ----
  {
    auto* f = make_group(QStringLiteral("容错与推理"));
    chkTol720_ = new QCheckBox(QStringLiteral("720p 图像启用容忍算法"), body);
    chkTol720_->setChecked(cfg_.tol_720p);
    chkTol720_->setToolTip(
        QStringLiteral("开启后：丢包造成的缺行用上一帧同一行补齐，\n"
                       "缺行不超过 5% 仍按正常帧处理（不再整帧丢弃）"));
    chkTolFusion_ = new QCheckBox(QStringLiteral("双目融合图启用容忍算法"), body);
    chkTolFusion_->setChecked(cfg_.tol_fusion);
    chkTolFusion_->setToolTip(QStringLiteral("同上，作用于 670×292 融合图（每包一行）"));
    chkInferComplete_ = new QCheckBox(QStringLiteral("只用完整帧推理"), body);
    chkInferComplete_->setChecked(cfg_.infer_only_complete);
    chkInferComplete_->setToolTip(
        QStringLiteral("仅在 720p 容忍算法开启时有意义：\n"
                       "补过行的帧只用于显示，不参与推理，保证识别精度"));
    f->addRow(chkTol720_);
    f->addRow(chkTolFusion_);
    f->addRow(chkInferComplete_);
  }

  // ---- 危险行为报警开关（放在逐类置信度上方）----
  // 只列实际会触发的行为：逆行算法层已禁用、越中心实线已并入压实线，两者都不再显示。
  // 勾选 = 该行为会报警并记入事件；取消勾选后规则完全不触发（不产生告警也不存事件）。
  {
    auto* g = new QGroupBox(QStringLiteral("危险行为报警开关"), body);
    auto* grid = new QGridLayout(g);
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(6);
    static const int kShow[6] = {
        static_cast<int>(DangerType::RedLightRunning),    // 闯红灯
        static_cast<int>(DangerType::CrossSolidLine),     // 压实线（含中心实线）
        static_cast<int>(DangerType::CrossDashedLine),    // 压虚线
        static_cast<int>(DangerType::WrongLane),          // 未按导引箭头行驶
        static_cast<int>(DangerType::CollisionWarning),   // 碰撞预警
        static_cast<int>(DangerType::LaneKeepViolation),  // 车道保持
    };
    // 开关名与报警文案可以不同：报警时显示 danger_name()（"请保持当前车道"），
    // 这里是开关项的名字，用简短的"车道保持"。
    static const char* kLabel[6] = {
        "闯红灯", "压实线", "压虚线", "未按导引箭头行驶", "碰撞预警", "车道保持",
    };
    for (int i = 0; i < 6; ++i) {
      const int t = kShow[i];
      auto* cell = new QWidget(g);
      auto* hb = new QHBoxLayout(cell);
      hb->setContentsMargins(0, 0, 0, 0);
      hb->setSpacing(6);
      auto* cb = new QCheckBox(QString::fromUtf8(kLabel[i]), cell);
      cb->setMinimumHeight(32);
      QFont cf = cb->font();
      cf.setPixelSize(15);
      cb->setFont(cf);
      cb->setChecked((cfg_.danger_enable_mask & (1u << t)) != 0u);
      dangerChk_[t] = cb;
      // 多帧平滑：勾选后该行为需连续满足去抖时间（默认 0.3s）才报警。
      // 只压制瞬时单帧抖动，条件中断立即撤销，不会延长报警。
      auto* sm = new QCheckBox(QStringLiteral("平滑"), cell);
      sm->setMinimumHeight(32);
      sm->setToolTip(QStringLiteral("勾选：连续 0.3 秒满足判据才报警（抑制单帧误报）\n"
                                    "不勾选：当帧满足即报警（单帧判定）"));
      QFont sf = sm->font();
      sf.setPixelSize(13);
      sm->setFont(sf);
      sm->setChecked((cfg_.danger_smooth_mask & (1u << t)) != 0u);
      smoothChk_[t] = sm;
      hb->addWidget(cb);
      hb->addStretch(1);
      hb->addWidget(sm);
      const int col = i / 3;   // 0..1 两列
      const int row = i % 3;
      grid->addWidget(cell, row, col);
    }
    auto* dtip = new QLabel(
        QStringLiteral("取消勾选的行为既不报警也不记录事件；\n"
                       "“平滑”= 该类行为连续满足 0.3 秒才报警，可逐项开启；\n"
                       "全部取消等于关闭危险判定。"),
        g);
    dtip->setWordWrap(true);
    grid->addWidget(dtip, 3, 0, 1, 2);
    lay->addWidget(g);
  }

  // ---- 逐类置信度：检测 16 类 / 分割 10 类 ----
  // 放在设置页最下方；未单独调整的类自动回落到上方统一阈值。
  {
    auto* g = new QGroupBox(QStringLiteral("各类别置信度（检测）"), body);
    auto* grid = new QGridLayout(g);
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(6);
    for (int i = 0; i < kNumClassesDet; ++i) {
      auto* lbl = new QLabel(QString::fromUtf8(det_class_name(i)), g);
      auto* sb = new QDoubleSpinBox(g);
      sb->setRange(0.05, 0.95);
      sb->setSingleStep(0.05);
      sb->setDecimals(2);
      sb->setMinimumHeight(32);
      const float v = cfg_.det_conf_by_cls[i];
      sb->setValue(v > 0.f ? static_cast<double>(v)
                           : (i < kNumLightClasses ? static_cast<double>(cfg_.conf_light)
                                                   : static_cast<double>(cfg_.conf_def)));
      detClsSpins_[i] = sb;
      const int col = (i / 8) * 2;
      const int row = i % 8;
      grid->addWidget(lbl, row, col);
      grid->addWidget(sb, row, col + 1);
    }
    lay->addWidget(g);
  }
  {
    auto* g = new QGroupBox(QStringLiteral("各类别置信度（分割）"), body);
    auto* grid = new QGridLayout(g);
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(6);
    static const char* kSegNames[kNumClassesSeg] = {"中心虚线", "中心实线", "车道边界",
                                                    "左转",     "右转",     "停止线",
                                                    "直左",     "直右",     "直行",
                                                    "人行横道"};
    for (int i = 0; i < kNumClassesSeg; ++i) {
      auto* lbl = new QLabel(QString::fromUtf8(kSegNames[i]), g);
      auto* sb = new QDoubleSpinBox(g);
      sb->setRange(0.05, 0.95);
      sb->setSingleStep(0.05);
      sb->setDecimals(2);
      sb->setMinimumHeight(32);
      const float v = cfg_.seg_conf_by_cls[i];
      sb->setValue(v > 0.f ? static_cast<double>(v) : static_cast<double>(cfg_.conf_seg));
      segClsSpins_[i] = sb;
      const int col = (i / 5) * 2;
      const int row = i % 5;
      grid->addWidget(lbl, row, col);
      grid->addWidget(sb, row, col + 1);
    }
    lay->addWidget(g);
  }

  auto* btnAlignAll = new QPushButton(QStringLiteral("各类别阈值对齐上方统一值"), body);
  btnAlignAll->setMinimumHeight(36);
  btnAlignAll->setStyleSheet(gray_button_qss());
  lay->addWidget(btnAlignAll);
  connect(btnAlignAll, &QPushButton::clicked, this, [this]() {
    for (int i = 0; i < kNumClassesDet; ++i) {
      detClsSpins_[i]->setValue(i < kNumLightClasses ? spConfLight_->value()
                                                     : spConfDef_->value());
    }
    for (int i = 0; i < kNumClassesSeg; ++i) segClsSpins_[i]->setValue(spConfSeg_->value());
  });

  // body 不加 stretch：加了会让 sizeHint 高度塌到视口高度，
  // QFormLayout 于是压缩各行，导致设置项显示不全。
  body->setMinimumWidth(520);

  // 统一控件行高，避免被布局压缩导致文字显示不全
  for (QDoubleSpinBox* s : body->findChildren<QDoubleSpinBox*>()) s->setMinimumHeight(34);
  for (QSpinBox* s : body->findChildren<QSpinBox*>()) s->setMinimumHeight(34);
  for (QLineEdit* s : body->findChildren<QLineEdit*>()) s->setMinimumHeight(34);
  for (QCheckBox* s : body->findChildren<QCheckBox*>()) s->setMinimumHeight(30);

  // 可滚动内容区
  auto* scroll = new QScrollArea(settingsPanel_);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  scroll->setWidget(body);
  outer->addWidget(scroll, 1);

  // 底部固定按钮行：始终可见，不随内容滚动
  auto* btnRow = new QHBoxLayout();
  auto* btnApply = new QPushButton(QStringLiteral("应用并保存"), settingsPanel_);
  auto* btnCancel = new QPushButton(QStringLiteral("关闭"), settingsPanel_);
  for (QPushButton* b : {btnApply, btnCancel}) {
    b->setMinimumHeight(44);
    QFont bf = b->font();
    bf.setPixelSize(17);
    b->setFont(bf);
    b->setStyleSheet(gray_button_qss());
  }
  btnRow->addWidget(btnApply, 2);
  btnRow->addWidget(btnCancel, 1);
  outer->addLayout(btnRow);

  connect(btnApply, &QPushButton::clicked, this, [this]() { applyFromSettings(true); });
  connect(btnCancel, &QPushButton::clicked, this, [this]() { settingsPanel_->hide(); });
  connect(btnHide, &QPushButton::clicked, this, [this]() { settingsPanel_->hide(); });

  settingsPanel_->hide();   // 默认隐藏，顶部“设置”按钮可开
}

void MainWindow::positionSettingsPanel() {
  if (!settingsPanel_) return;
  // 面板宽度：取屏宽的 1/3，但不小于 620、不大于 820
  int w = width() / 3;
  if (w < 620) w = 620;
  if (w > 820) w = 820;
  if (w > width()) w = width();
  // 从顶部状态栏下方开始，避免遮住“设置/退出”按钮与运行状态；
  // 面板停靠屏幕左侧。
  const int top = 60;
  const int h = (height() > top) ? (height() - top) : height();
  settingsPanel_->setGeometry(0, top, w, h);
}

void MainWindow::toggleSettingsPanel() {
  if (!settingsPanel_) return;
  const bool show = !settingsPanel_->isVisible();
  settingsPanel_->setVisible(show);
  if (show) {
    positionSettingsPanel();
    settingsPanel_->raise();
  }
}

// ---------------- 模拟导航悬浮窗 ----------------
void MainWindow::buildNavPanel() {
  navPanel_ = new QWidget(this);
  navPanel_->setObjectName(QStringLiteral("navPanel"));
  navPanel_->setAttribute(Qt::WA_StyledBackground, true);
  navPanel_->setStyleSheet(QStringLiteral(
      "QWidget#navPanel{background:#1f2630;border:2px solid #3d4d63;border-radius:8px;}"));

  auto* lay = new QVBoxLayout(navPanel_);
  lay->setContentsMargins(12, 10, 12, 12);
  lay->setSpacing(10);

  auto* head = new QHBoxLayout();
  auto* titleLbl = new QLabel(QStringLiteral("模拟导航"), navPanel_);
  {
    QFont f = titleLbl->font();
    f.setPixelSize(19);
    f.setBold(true);
    titleLbl->setFont(f);
  }
  head->addWidget(titleLbl);
  head->addStretch(1);
  auto* btnClose = new QPushButton(QStringLiteral("收起"), navPanel_);
  btnClose->setMinimumHeight(32);
  btnClose->setMinimumWidth(80);
  btnClose->setStyleSheet(gray_button_qss());
  head->addWidget(btnClose);
  lay->addLayout(head);

  auto* hint = new QLabel(QStringLiteral("点击设定导航方向，再次点击取消"), navPanel_);
  hint->setStyleSheet(QStringLiteral("color:#8a919c;"));
  lay->addWidget(hint);

  static const char* kNames[7] = {"前进", "停车", "左转", "右转", "调头", "向左变道", "向右变道"};
  auto* grid = new QGridLayout();
  grid->setSpacing(8);
  for (int i = 0; i < 7; ++i) {
    auto* b = new QPushButton(QString::fromUtf8(kNames[i]), navPanel_);
    b->setCheckable(true);
    b->setMinimumHeight(58);
    QFont f = b->font();
    f.setPixelSize(18);
    f.setBold(true);
    b->setFont(f);
    b->setStyleSheet(QStringLiteral(
        "QPushButton{background:#e8ebef;color:#20242a;border:1px solid #b6bcc4;"
        "border-radius:6px;}"
        "QPushButton:hover{background:#f4f6f8;}"
        "QPushButton:pressed{background:#d2d7dd;}"
        "QPushButton:checked{background:#8f97a1;color:#fff;border:2px solid #5f6771;}"));
    navBtns_[i] = b;
    grid->addWidget(b, i / 2, i % 2);
    const int cmd = i + 1;
    connect(b, &QPushButton::clicked, this, [this, cmd]() { setNavCmd(cmd); });
  }
  lay->addLayout(grid);
  lay->addStretch(1);

  connect(btnClose, &QPushButton::clicked, this, [this]() { navPanel_->hide(); });

  navPanel_->hide();
}

void MainWindow::positionNavPanel() {
  if (!navPanel_) return;
  const int w = 430;
  const int h = 400;
  const int x = 20;
  const int y = 80;
  navPanel_->setGeometry(x, y, w, h);
}

void MainWindow::toggleNavPanel() {
  if (!navPanel_) return;
  const bool show = !navPanel_->isVisible();
  navPanel_->setVisible(show);
  if (show) {
    positionNavPanel();
    navPanel_->raise();
  }
}

void MainWindow::setNavCmd(int cmd) {
  // 自锁：再次点击同一项 -> 取消
  const int cur = pipeline_.nav_cmd();
  const int next = (cur == cmd) ? kNavNone : cmd;
  pipeline_.set_nav_cmd(next);
  for (int i = 0; i < 7; ++i) navBtns_[i]->setChecked((i + 1) == next);
  btnNav_->setText(next == kNavNone
                       ? QStringLiteral("导航")
                       : QStringLiteral("导航：") + QString::fromUtf8(nav_cmd_name(next)));
}

void MainWindow::applyFromSettings(bool restart) {
  cfg_.udp_port = spUdpPort_->value();
  cfg_.main_cam = spMainCam_->value();
  pipeline_.set_main_cam(cfg_.main_cam);   // 实时生效（四路模式取景区域）
  cfg_.bind_ip = edBind_->text().trimmed().toStdString();
  cfg_.conf_light = static_cast<float>(spConfLight_->value());
  cfg_.conf_def = static_cast<float>(spConfDef_->value());
  cfg_.conf_seg = static_cast<float>(spConfSeg_->value());
  cfg_.iou = static_cast<float>(spIou_->value());
  cfg_.iou_seg = static_cast<float>(spIouSeg_->value());
  cfg_.bev_fwd_max = static_cast<float>(spFwdMax_->value());
  cfg_.bev_lat_half = static_cast<float>(spLatHalf_->value());
  cfg_.bev_lane_min_m = static_cast<float>(spLaneMin_->value());
  cfg_.infer_every = spInferEvery_->value();
  cfg_.cam_fx = static_cast<float>(spCamFx_->value());
  cfg_.cam_fy = static_cast<float>(spCamFy_->value());
  cfg_.cam_cx = static_cast<float>(spCamCx_->value());
  cfg_.cam_cy = static_cast<float>(spCamCy_->value());
  cfg_.cam_height = static_cast<float>(spCamHeight_->value());
  cfg_.cam_to_front = static_cast<float>(spCamToFront_->value());
  cfg_.cam_pitch_deg = static_cast<float>(spCamPitch_->value());
  cfg_.cam_offset_y = static_cast<float>(spCamOffsetY_->value());
  cfg_.cam_yaw_deg = static_cast<float>(spCamYaw_->value());
  cfg_.cam_k1 = static_cast<float>(spCamK1_->value());
  cfg_.cam_k2 = static_cast<float>(spCamK2_->value());
  // 报警开关 -> 位掩码（没有复选框的项保持原值，避免被清掉）
  {
    uint32_t mask = cfg_.danger_enable_mask;
    for (int t = 1; t <= 7; ++t) {
      if (!dangerChk_[t]) continue;
      if (dangerChk_[t]->isChecked())
        mask |= (1u << t);
      else
        mask &= ~(1u << t);
    }
    cfg_.danger_enable_mask = mask;
  }
  // 多帧平滑开关 -> 位掩码
  {
    uint32_t mask = cfg_.danger_smooth_mask;
    for (int t = 1; t <= 7; ++t) {
      if (!smoothChk_[t]) continue;
      if (smoothChk_[t]->isChecked())
        mask |= (1u << t);
      else
        mask &= ~(1u << t);
    }
    cfg_.danger_smooth_mask = mask;
  }
  cfg_.events_dir = edEventsDir_->text().trimmed().toStdString();
  cfg_.event_max_mb = static_cast<uint64_t>(spMaxMb_->value());
  cfg_.upload_url = edUpload_->text().trimmed().toStdString();
  // 容错与推理策略（实时生效，不需重启流水线）
  if (chkTol720_) cfg_.tol_720p = chkTol720_->isChecked();
  if (chkTolFusion_) cfg_.tol_fusion = chkTolFusion_->isChecked();
  if (chkInferComplete_) cfg_.infer_only_complete = chkInferComplete_->isChecked();

  // 逐类置信度
  for (int i = 0; i < kNumClassesDet; ++i)
    cfg_.det_conf_by_cls[i] = static_cast<float>(detClsSpins_[i]->value());
  for (int i = 0; i < kNumClassesSeg; ++i)
    cfg_.seg_conf_by_cls[i] = static_cast<float>(segClsSpins_[i]->value());

  saveSettings();        // 持久化，下次启动自动恢复

  pipeline_.update_live(cfg_);

  if (restart && running_) {
    pipeline_.stop();
    std::string err;
    if (!pipeline_.start(cfg_, &err)) {
      statusRun_->setText(QStringLiteral("● 启动失败：") + QString::fromStdString(err));
      running_ = false;
    }
  }
}

void MainWindow::loadSettings() {
  QSettings s(QString::fromUtf8(kSettingsFile), QSettings::IniFormat);
  if (s.status() != QSettings::NoError) return;
  s.beginGroup(QStringLiteral("pipeline"));

  cfg_.udp_port = s.value(QStringLiteral("udp_port"), cfg_.udp_port).toInt();
  cfg_.main_cam = s.value(QStringLiteral("main_cam"), cfg_.main_cam).toInt();
  cfg_.bind_ip = s.value(QStringLiteral("bind_ip"), QString::fromStdString(cfg_.bind_ip))
                     .toString()
                     .toStdString();

  cfg_.conf_light = s.value(QStringLiteral("conf_light"), static_cast<double>(cfg_.conf_light))
                        .toFloat();
  cfg_.conf_def = s.value(QStringLiteral("conf_def"), static_cast<double>(cfg_.conf_def)).toFloat();
  cfg_.conf_seg = s.value(QStringLiteral("conf_seg"), static_cast<double>(cfg_.conf_seg)).toFloat();
  cfg_.iou = s.value(QStringLiteral("iou"), static_cast<double>(cfg_.iou)).toFloat();
  cfg_.iou_seg = s.value(QStringLiteral("iou_seg"), static_cast<double>(cfg_.iou_seg)).toFloat();

  cfg_.bev_fwd_max =
      s.value(QStringLiteral("bev_fwd_max"), static_cast<double>(cfg_.bev_fwd_max)).toFloat();
  cfg_.bev_lat_half =
      s.value(QStringLiteral("bev_lat_half"), static_cast<double>(cfg_.bev_lat_half)).toFloat();
  cfg_.bev_lane_min_m =
      s.value(QStringLiteral("bev_lane_min_m"), static_cast<double>(cfg_.bev_lane_min_m)).toFloat();
  cfg_.infer_every = s.value(QStringLiteral("infer_every"), cfg_.infer_every).toInt();

  cfg_.cam_fx = s.value(QStringLiteral("cam_fx"), static_cast<double>(cfg_.cam_fx)).toFloat();
  cfg_.cam_fy = s.value(QStringLiteral("cam_fy"), static_cast<double>(cfg_.cam_fy)).toFloat();
  cfg_.cam_cx = s.value(QStringLiteral("cam_cx"), static_cast<double>(cfg_.cam_cx)).toFloat();
  cfg_.cam_cy = s.value(QStringLiteral("cam_cy"), static_cast<double>(cfg_.cam_cy)).toFloat();
  cfg_.cam_height =
      s.value(QStringLiteral("cam_height"), static_cast<double>(cfg_.cam_height)).toFloat();
  cfg_.cam_to_front =
      s.value(QStringLiteral("cam_to_front"), static_cast<double>(cfg_.cam_to_front)).toFloat();
  cfg_.cam_pitch_deg =
      s.value(QStringLiteral("cam_pitch_deg"), static_cast<double>(cfg_.cam_pitch_deg)).toFloat();
  cfg_.cam_offset_y =
      s.value(QStringLiteral("cam_offset_y"), static_cast<double>(cfg_.cam_offset_y)).toFloat();
  cfg_.cam_yaw_deg =
      s.value(QStringLiteral("cam_yaw_deg"), static_cast<double>(cfg_.cam_yaw_deg)).toFloat();
  cfg_.danger_enable_mask = static_cast<uint32_t>(
      s.value(QStringLiteral("danger_enable_mask"),
              static_cast<qulonglong>(cfg_.danger_enable_mask))
          .toULongLong());
  // 兼容旧配置：新增"车道保持"位时做一次性迁移。
  // 用版本标记区分"旧版写下的掩码"和"用户主动改过的掩码"，只迁移一次。
  if (!s.contains(QStringLiteral("danger_mask_ver"))) {
    cfg_.danger_enable_mask |= (1u << static_cast<unsigned>(DangerType::LaneKeepViolation));
    s.setValue(QStringLiteral("danger_mask_ver"), 2);
  }
  cfg_.danger_smooth_mask = static_cast<uint32_t>(
      s.value(QStringLiteral("danger_smooth_mask"),
              static_cast<qulonglong>(cfg_.danger_smooth_mask))
          .toULongLong());
  cfg_.cam_k1 =
      s.value(QStringLiteral("cam_k1"), static_cast<double>(cfg_.cam_k1)).toFloat();
  cfg_.cam_k2 =
      s.value(QStringLiteral("cam_k2"), static_cast<double>(cfg_.cam_k2)).toFloat();

  cfg_.events_dir = s.value(QStringLiteral("events_dir"), QString::fromStdString(cfg_.events_dir))
                        .toString()
                        .toStdString();
  cfg_.event_max_mb = s.value(QStringLiteral("event_max_mb"),
                              static_cast<qulonglong>(cfg_.event_max_mb))
                          .toULongLong();
  cfg_.upload_url = s.value(QStringLiteral("upload_url"), QString::fromStdString(cfg_.upload_url))
                        .toString()
                        .toStdString();

  // 容错与推理策略
  cfg_.tol_720p = s.value(QStringLiteral("tol_720p"), cfg_.tol_720p).toBool();
  cfg_.tol_fusion = s.value(QStringLiteral("tol_fusion"), cfg_.tol_fusion).toBool();
  cfg_.infer_only_complete =
      s.value(QStringLiteral("infer_only_complete"), cfg_.infer_only_complete).toBool();
  cfg_.fusion565_bigendian =
      s.value(QStringLiteral("fusion565_bigendian"), cfg_.fusion565_bigendian).toBool();

  // 逐类置信度
  for (int i = 0; i < kNumClassesDet; ++i) {
    const QString k = QStringLiteral("det_conf_%1").arg(i);
    cfg_.det_conf_by_cls[i] =
        s.value(k, static_cast<double>(cfg_.det_conf_by_cls[i])).toFloat();
  }
  for (int i = 0; i < kNumClassesSeg; ++i) {
    const QString k = QStringLiteral("seg_conf_%1").arg(i);
    cfg_.seg_conf_by_cls[i] =
        s.value(k, static_cast<double>(cfg_.seg_conf_by_cls[i])).toFloat();
  }

  s.endGroup();
}

void MainWindow::saveSettings() {
  QSettings s(QString::fromUtf8(kSettingsFile), QSettings::IniFormat);
  s.beginGroup(QStringLiteral("pipeline"));

  s.setValue(QStringLiteral("udp_port"), cfg_.udp_port);
  s.setValue(QStringLiteral("main_cam"), cfg_.main_cam);
  s.setValue(QStringLiteral("bind_ip"), QString::fromStdString(cfg_.bind_ip));

  s.setValue(QStringLiteral("conf_light"), static_cast<double>(cfg_.conf_light));
  s.setValue(QStringLiteral("conf_def"), static_cast<double>(cfg_.conf_def));
  s.setValue(QStringLiteral("conf_seg"), static_cast<double>(cfg_.conf_seg));
  s.setValue(QStringLiteral("iou"), static_cast<double>(cfg_.iou));
  s.setValue(QStringLiteral("iou_seg"), static_cast<double>(cfg_.iou_seg));

  s.setValue(QStringLiteral("bev_fwd_max"), static_cast<double>(cfg_.bev_fwd_max));
  s.setValue(QStringLiteral("bev_lat_half"), static_cast<double>(cfg_.bev_lat_half));
  s.setValue(QStringLiteral("bev_lane_min_m"), static_cast<double>(cfg_.bev_lane_min_m));
  s.setValue(QStringLiteral("infer_every"), cfg_.infer_every);

  s.setValue(QStringLiteral("cam_fx"), static_cast<double>(cfg_.cam_fx));
  s.setValue(QStringLiteral("cam_fy"), static_cast<double>(cfg_.cam_fy));
  s.setValue(QStringLiteral("cam_cx"), static_cast<double>(cfg_.cam_cx));
  s.setValue(QStringLiteral("cam_cy"), static_cast<double>(cfg_.cam_cy));
  s.setValue(QStringLiteral("cam_height"), static_cast<double>(cfg_.cam_height));
  s.setValue(QStringLiteral("cam_to_front"), static_cast<double>(cfg_.cam_to_front));
  s.setValue(QStringLiteral("cam_pitch_deg"), static_cast<double>(cfg_.cam_pitch_deg));
  s.setValue(QStringLiteral("cam_offset_y"), static_cast<double>(cfg_.cam_offset_y));
  s.setValue(QStringLiteral("cam_yaw_deg"), static_cast<double>(cfg_.cam_yaw_deg));
  s.setValue(QStringLiteral("danger_enable_mask"),
             static_cast<qulonglong>(cfg_.danger_enable_mask));
  s.setValue(QStringLiteral("danger_smooth_mask"),
             static_cast<qulonglong>(cfg_.danger_smooth_mask));
  s.setValue(QStringLiteral("cam_k1"), static_cast<double>(cfg_.cam_k1));
  s.setValue(QStringLiteral("cam_k2"), static_cast<double>(cfg_.cam_k2));

  s.setValue(QStringLiteral("events_dir"), QString::fromStdString(cfg_.events_dir));
  s.setValue(QStringLiteral("event_max_mb"), static_cast<qulonglong>(cfg_.event_max_mb));
  s.setValue(QStringLiteral("upload_url"), QString::fromStdString(cfg_.upload_url));

  // 容错与推理策略
  s.setValue(QStringLiteral("tol_720p"), cfg_.tol_720p);
  s.setValue(QStringLiteral("tol_fusion"), cfg_.tol_fusion);
  s.setValue(QStringLiteral("infer_only_complete"), cfg_.infer_only_complete);
  s.setValue(QStringLiteral("fusion565_bigendian"), cfg_.fusion565_bigendian);

  // 逐类置信度
  for (int i = 0; i < kNumClassesDet; ++i) {
    s.setValue(QStringLiteral("det_conf_%1").arg(i),
               static_cast<double>(cfg_.det_conf_by_cls[i]));
  }
  for (int i = 0; i < kNumClassesSeg; ++i) {
    s.setValue(QStringLiteral("seg_conf_%1").arg(i),
               static_cast<double>(cfg_.seg_conf_by_cls[i]));
  }

  s.endGroup();
  s.sync();
}

void MainWindow::saveCurrentSettings() {
  // 控件未创建时（极早期退出）跳过
  if (!spUdpPort_ || !edUpload_) return;
  applyFromSettings(false);
}

void MainWindow::resizeEvent(QResizeEvent* e) {
  QMainWindow::resizeEvent(e);
  if (settingsPanel_ && settingsPanel_->isVisible()) positionSettingsPanel();
  if (navPanel_ && navPanel_->isVisible()) positionNavPanel();
}

void MainWindow::closeEvent(QCloseEvent* e) {
  saveCurrentSettings();
  QMainWindow::closeEvent(e);
}

void MainWindow::setRunning(bool on) {
  if (on == running_) return;
  if (on) {
    std::string err;
    if (!pipeline_.start(cfg_, &err)) {
      statusRun_->setText(QStringLiteral("● 启动失败：") + QString::fromStdString(err));
      return;
    }
    running_ = true;
    statusRun_->setText(QStringLiteral("● 运行中"));
    btnStart_->setText(QStringLiteral("停止"));
  } else {
    pipeline_.stop();
    running_ = false;
    statusRun_->setText(QStringLiteral("● 已停止"));
    btnStart_->setText(QStringLiteral("开始"));
  }
}

void MainWindow::cycleViewMode() {
  switch (viewMode_) {
    case ViewMode::Annotated: viewMode_ = ViewMode::Raw; break;
    case ViewMode::Raw:       viewMode_ = ViewMode::Fusion; break;
    default:                  viewMode_ = ViewMode::Annotated; break;
  }
  applyViewMode();
}

void MainWindow::applyViewMode() {
  video_->setViewMode(viewMode_);
  // pipeline 侧：只有 Fusion 模式才接收/缓存融合包；只有 Raw 模式才额外拷整幅原图
  pipeline_.set_view_mode(viewMode_);
  pipeline_.set_show_raw(viewMode_ == ViewMode::Raw);

  const char* txt = (viewMode_ == ViewMode::Annotated) ? "识别结果"
                    : (viewMode_ == ViewMode::Raw)     ? "原图"
                                                       : "双目融合";
  btnView_->setText(QString::fromUtf8(txt));

  // 切到融合模式时先清掉旧图，等待新帧（否则会显示上次的残留画面）
  if (viewMode_ == ViewMode::Fusion) {
    fusionHasImg_ = false;
    lastFusionMs_ = 0;
    video_->setFusionImage(QImage());
  }
}

void MainWindow::onTick() {
  const int64_t now_ms = QDateTime::currentMSecsSinceEpoch();
  FrameView fv;
  const bool got = pipeline_.poll(&fv);
  if (got) {
    lastCamIdx_ = fv.cam_index;
    lastInferMs_ = fv.infer_ms;
    lastFpsImg_ = fv.fps_img;
    lastFpsFusion_ = fv.fps_fusion;
  }

  // ---- 双目融合显示：有新帧才重建 QImage（避免每 33ms 拷 0.6MB）----
  if (viewMode_ == ViewMode::Fusion) {
    bool updated = false;
    if (got && fv.fusion_seq != lastFusionSeq_ && fv.fusion_w > 0 && fv.fusion_h > 0 &&
        fv.fusion.size() >= static_cast<size_t>(fv.fusion_w) * fv.fusion_h * 3u) {
      lastFusionSeq_ = fv.fusion_seq;
      lastFusionMs_ = now_ms;
      const QImage im(fv.fusion.data(), fv.fusion_w, fv.fusion_h, fv.fusion_w * 3,
                      QImage::Format_RGB888);
      fusionImg_ = im.copy();
      updated = true;
    }
    const bool has = !fusionImg_.isNull() && (now_ms - lastFusionMs_ <= 1200);
    if (updated) {
      // 关键：每个新帧都要推给控件，否则画面会永远停在第 1 帧
      fusionHasImg_ = true;
      video_->setFusionImage(fusionImg_);
    } else if (has != fusionHasImg_) {
      fusionHasImg_ = has;
      video_->setFusionImage(has ? fusionImg_ : QImage());
    }
    // 融合模式显示融合图自己的收包帧率（不受 720p 帧是否到达影响）
    statusFps_->setText(QStringLiteral("融合  %1fps").arg(lastFpsFusion_, 0, 'f', 1));
  } else {
    // 720p 收包帧率（按包数换算，与丢包/容忍/显示无关）
    statusFps_->setText(QStringLiteral("CAM%1  %2fps  %3ms")
                            .arg(lastCamIdx_ + 1)
                            .arg(lastFpsImg_, 0, 'f', 1)
                            .arg(lastInferMs_));
  }

  if (!got) return;

  // 只取当前显示模式需要的那一路，避免每帧多余的内存拷贝
  if (viewMode_ == ViewMode::Annotated) {
    if (fv.w > 0 && fv.h > 0 && !fv.rgb.empty()) {
      const QImage img(fv.rgb.data(), fv.w, fv.h, fv.w * 3, QImage::Format_RGB888);
      video_->setImage(img.copy());   // 深拷贝，避免引用 worker 线程缓冲
      video_->setDetections(fv.dets);
      video_->setLanes(fv.lanes_img);
    }
  } else if (viewMode_ == ViewMode::Raw && fv.raw_w > 0 && fv.raw_h > 0 && !fv.raw.empty()) {
    // 原始整幅画面（四路输入时是四宫格 1280x720）
    const QImage raw(fv.raw.data(), fv.raw_w, fv.raw_h, fv.raw_w * 3, QImage::Format_RGB888);
    video_->setRawImage(raw.copy());
  }

  if (!fv.bev.cmds.empty()) bev_->setScene(fv.bev);

  // 驾驶员行为：由收包线程每帧刷新（30fps），不跟随推理帧，所以是实时的
  if (fv.behavior.empty() || fv.behavior == "无") {
    statusBehav_->setText(QStringLiteral("行为 —"));
    statusBehav_->setStyleSheet(QStringLiteral("color:#9aa0a8;"));
  } else {
    const QString b = QString::fromStdString(fv.behavior);
    statusBehav_->setText(QStringLiteral("行为 ") + b);
    // 有行为时用黑色加粗，保证在顶栏底色上足够醒目
    statusBehav_->setStyleSheet(QStringLiteral("color:#000000;font-weight:bold;"));
  }

  if (fv.danger.empty()) {
    statusDanger_->setText(QStringLiteral("无告警"));
    statusDanger_->setStyleSheet(QStringLiteral("color:#9aa0a8;"));
  } else {
    statusDanger_->setText(QStringLiteral("⚠ ") + QString::fromStdString(fv.danger));
    statusDanger_->setStyleSheet(QStringLiteral("color:#e43a3a;font-weight:bold;"));
  }

  // 导引/建议：绿色文字（过长时截断，避免把顶栏挤出屏幕）
  if (!fv.advice.empty()) {
    QString a = QString::fromStdString(fv.advice);
    if (a.size() > 12) a = a.left(12) + QStringLiteral("…");
    statusAdvice_->setText(QStringLiteral("导引：") + a);
    statusAdvice_->setStyleSheet(QStringLiteral("color:#2ecc71;font-weight:bold;"));
  } else {
    statusAdvice_->setText(QString());
    statusAdvice_->setStyleSheet(QString());
  }

  // 告警与导引同时叠加到 2D 影像上（告警红、导引绿）
  {
    const QString d = fv.danger.empty()
                          ? QString()
                          : QStringLiteral("⚠ ") + QString::fromStdString(fv.danger);
    const QString a = fv.advice.empty() ? QString()
                                        : QStringLiteral("导引：") +
                                              QString::fromStdString(fv.advice);
    video_->setOverlayText(d, a);
  }

  if (fv.ego_valid) {
    statusEgo_->setText(QStringLiteral("车道 %1/%2m")
                            .arg(fv.ego_width, 0, 'f', 2)
                            .arg(fv.ego_center, 0, 'f', 2));
  } else {
    statusEgo_->setText(QStringLiteral("本车道 —"));
  }

  statusEvent_->setText(QStringLiteral("事件 %1").arg(static_cast<qulonglong>(fv.event_count)));

  updateEventTable(fv);
}

void MainWindow::updateEventTable(const FrameView& fv) {
  QString key;
  if (!fv.events.empty()) {
    key = QStringLiteral("%1:%2").arg(static_cast<qulonglong>(fv.events.size()))
              .arg(static_cast<qulonglong>(fv.events.front().id));
  }
  if (key == lastEventKey_ && !fv.events.empty()) return;
  lastEventKey_ = key;

  events_->setRowCount(static_cast<int>(fv.events.size()));
  for (int i = 0; i < static_cast<int>(fv.events.size()); ++i) {
    const EventRow& r = fv.events[static_cast<size_t>(i)];
    events_->setItem(i, 0, new QTableWidgetItem(fmt_ts(r.ts_ms)));
    events_->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(r.type)));
    // 只显示数值，不标单位：各类行为的度量含义/单位不同
    // （压线=重叠面积 m²，闯红灯/碰撞=纵向距离 m，错道=横向偏差 m，车道保持=横向位置 m）
    events_->setItem(i, 2, new QTableWidgetItem(QStringLiteral("%1").arg(r.metric, 0, 'f', 2)));
    events_->setItem(i, 3, new QTableWidgetItem(QStringLiteral("%1").arg(r.conf, 0, 'f', 2)));
    events_->setItem(i, 4,
                     new QTableWidgetItem(r.uploaded ? QStringLiteral("已上传")
                                                     : QStringLiteral("待上传")));
  }
}

} // namespace adas
