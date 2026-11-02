#pragma once
#include "common/types.h"
#include "ui/pipeline.h"   // LanePoly

#include <QImage>
#include <QString>
#include <QWidget>
#include <vector>

namespace adas {

// 左侧视频区：显示主摄画面，可选择叠加识别结果（检测框）
class VideoWidget : public QWidget {
public:
  explicit VideoWidget(QWidget* parent = nullptr);

  void setImage(const QImage& img);
  void setRawImage(const QImage& img);   // 原图（“显示原图”模式用，可为空）
  void setFusionImage(const QImage& img); // 双目融合图（为空 = 无信号）
  void setDetections(const std::vector<Detection>& dets);
  void setLanes(const std::vector<LanePoly>& lanes);
  // 显示模式：识别结果 / 原图 / 双目融合（循环切换）
  void setViewMode(ViewMode m) { mode_ = m; update(); }
  ViewMode viewMode() const { return mode_; }
  // 2D 影像上的文字叠加：告警（红）与驾驶导引（绿）
  void setOverlayText(const QString& danger, const QString& advice);
  bool annotated() const { return mode_ == ViewMode::Annotated; }
  void clear();

protected:
  void paintEvent(QPaintEvent* e) override;

private:
  QImage img_;      // 主摄画面（识别叠加用）
  QImage raw_;      // 原始整幅画面（四路=四宫格）
  QImage fusion_;   // 双目融合图（为空 = 无信号）
  std::vector<Detection> dets_;
  std::vector<LanePoly>  lanes_;
  ViewMode mode_ = ViewMode::Annotated;
  QString danger_text_;   // 叠加：危险告警（红）
  QString advice_text_;   // 叠加：驾驶导引（绿）
};

// 检测模型类别名（16 类）
const char* det_class_name(int cls);

} // namespace adas
