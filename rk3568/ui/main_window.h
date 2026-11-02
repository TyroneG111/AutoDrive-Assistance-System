#pragma once
#include "ui/pipeline.h"

#include <QMainWindow>
#include <QImage>
#include <QString>

class QLabel;
class QPushButton;
class QTableWidget;
class QTimer;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QLineEdit;
class QWidget;
class QResizeEvent;
class QCloseEvent;

namespace adas {

class VideoWidget;
class BevWidget;

// 1920x1080 主界面：左侧视频 / 右侧 BEV / 底部事件列表 / 右侧设置抽屉
class MainWindow : public QMainWindow {
public:
  explicit MainWindow(const PipelineConfig& cfg, QWidget* parent = nullptr);
  ~MainWindow() override;

  void setIconDir(const QString& dir);

  // 把控件当前值写回配置并落盘（供 closeEvent / aboutToQuit / 信号退出调用）
  void saveCurrentSettings();

protected:
  // 退出时把当前控件里的值落盘：保证“每次退出保留用户设置”，
  // 即使没点“应用”也不会丢。
  void closeEvent(QCloseEvent* e) override;
  // 覆盖式设置面板跟随窗口尺寸重定位
  void resizeEvent(QResizeEvent* e) override;

private:
  void buildUi();
  void buildSettingsPanel();
  void buildNavPanel();
  void positionSettingsPanel();
  void positionNavPanel();
  void toggleSettingsPanel();
  void toggleNavPanel();
  void cycleViewMode();          // 识别结果 → 原图 → 双目融合 → …
  void applyViewMode();
  void setNavCmd(int cmd);
  void applyFromSettings(bool restart);
  void loadSettings();
  void saveSettings();
  void onTick();
  void setRunning(bool on);
  void updateEventTable(const FrameView& fv);

  Pipeline pipeline_;
  PipelineConfig cfg_;

  VideoWidget* video_ = nullptr;
  BevWidget*   bev_ = nullptr;
  QTableWidget* events_ = nullptr;
  QTimer* timer_ = nullptr;

  QLabel* statusRun_ = nullptr;
  QLabel* statusFps_ = nullptr;
  QLabel* statusBehav_ = nullptr;   // 驾驶员行为（实时，30fps 刷新）
  QLabel* statusDanger_ = nullptr;
  QLabel* statusAdvice_ = nullptr;
  QLabel* statusEgo_ = nullptr;
  QLabel* statusEvent_ = nullptr;

  QPushButton* btnStart_ = nullptr;
  QPushButton* btnView_ = nullptr;
  QPushButton* btnSettings_ = nullptr;
  QPushButton* btnNav_ = nullptr;
  QPushButton* btnQuit_ = nullptr;
  // 覆盖式设置面板：直接挂在 MainWindow 上、手动定位，
  // 不参与 QMainWindow 布局，打开/关闭不会影响视频与 BEV。
  QWidget* settingsPanel_ = nullptr;
  // 模拟导航悬浮窗
  QWidget* navPanel_ = nullptr;
  QPushButton* navBtns_[7] = {};   // 下标 0..6 对应 NavCmd 1..7

  // 设置控件
  QSpinBox*      spUdpPort_ = nullptr;
  QSpinBox*      spMainCam_ = nullptr;
  QLineEdit*     edBind_ = nullptr;
  QDoubleSpinBox* spConfLight_ = nullptr;
  QDoubleSpinBox* spConfDef_ = nullptr;
  QDoubleSpinBox* spConfSeg_ = nullptr;
  QDoubleSpinBox* spIou_ = nullptr;
  QDoubleSpinBox* spIouSeg_ = nullptr;
  QDoubleSpinBox* spFwdMax_ = nullptr;
  QDoubleSpinBox* spLatHalf_ = nullptr;
  QDoubleSpinBox* spLaneMin_ = nullptr;
  QSpinBox*      spInferEvery_ = nullptr;
  QDoubleSpinBox* spCamFx_ = nullptr;
  QDoubleSpinBox* spCamFy_ = nullptr;
  QDoubleSpinBox* spCamCx_ = nullptr;
  QDoubleSpinBox* spCamCy_ = nullptr;
  QDoubleSpinBox* spCamHeight_ = nullptr;
  QDoubleSpinBox* spCamToFront_ = nullptr;
  QDoubleSpinBox* spCamPitch_ = nullptr;
  QDoubleSpinBox* spCamOffsetY_ = nullptr;
  QDoubleSpinBox* spCamYaw_ = nullptr;
  QDoubleSpinBox* spCamK1_ = nullptr;
  QDoubleSpinBox* spCamK2_ = nullptr;
  QLineEdit*     edEventsDir_ = nullptr;
  QSpinBox*      spMaxMb_ = nullptr;
  QLineEdit*     edUpload_ = nullptr;

  // 容错与推理策略
  QCheckBox* chkTol720_ = nullptr;        // 720p 图像启用容忍算法
  QCheckBox* chkTolFusion_ = nullptr;     // 双目融合图启用容忍算法
  QCheckBox* chkInferComplete_ = nullptr; // 只用完整帧推理

  // 危险行为报警开关（下标 1..7 对应 DangerType，0 不用）
  QCheckBox* dangerChk_[8] = {};
  // 每类报警的多帧平滑开关（勾选 = 连续满足 0.3s 才报警）
  QCheckBox* smoothChk_[8] = {};

  // 逐类置信度（设置页最下方）
  QDoubleSpinBox* detClsSpins_[kNumClassesDet] = {};
  QDoubleSpinBox* segClsSpins_[kNumClassesSeg] = {};

  QString lastEventKey_;
  bool    running_ = false;

  // ---- 显示模式 / 双目融合图 ----
  ViewMode viewMode_ = ViewMode::Annotated;
  QImage   fusionImg_;          // 最近一帧融合图（仅在 fusion_seq 变化时重建）
  uint64_t lastFusionSeq_ = 0;
  int64_t  lastFusionMs_ = 0;   // 最近一次收到融合帧的时间（判“无信号”）
  bool     fusionHasImg_ = false;

  // 顶栏帧率：用 pipeline 按**收包包数**换算的帧率（fps_img / fps_fusion），
  // 与丢包、容忍算法、显示节拍无关，反映 FPGA 实际出图帧率。
  int    lastCamIdx_ = 0;     // 最近一帧的主摄序号（当前帧没有新帧时顶栏用）
  double lastInferMs_ = 0.0;  // 最近一帧的推理耗时
  double lastFpsImg_ = 0.0;   // 最近一次拿到的 720p 收包帧率
  double lastFpsFusion_ = 0.0;// 最近一次拿到的融合图收包帧率
};

} // namespace adas
