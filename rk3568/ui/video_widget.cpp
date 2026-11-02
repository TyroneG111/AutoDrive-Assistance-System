#include "ui/video_widget.h"

#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QPaintEvent>
#include <QPalette>
#include <QPen>
#include <QPolygonF>
#include <QRect>
#include <QString>

namespace adas {

const char* det_class_name(int cls) {
  static const char* kNames[16] = {
      "红灯(圆)", "红灯(左)", "红灯(右)", "红灯(直)", "绿灯(圆)", "绿灯(左)",
      "绿灯(右)", "绿灯(直)", "黄灯(圆)", "黄灯(左)", "黄灯(右)", "黄灯(直)",
      "行人",     "两/三轮",   "轿车",     "大型车"};
  if (cls < 0 || cls >= 16) return "未知";
  return kNames[cls];
}

namespace {

QColor class_color(int cls) {
  if (cls < 12) return QColor(255, 214, 0);      // 信号灯
  if (cls == 12) return QColor(255, 64, 64);     // 行人
  if (cls == 13) return QColor(255, 140, 0);     // 两/三轮
  return QColor(64, 220, 120);                   // 车辆
}

// 分割模型 10 类配色（与方法表 segment_B3_10cls.yaml 一致）
QColor seg_color(int cls) {
  switch (cls) {
    case 0: return QColor(0, 220, 255);    // 中心虚线   青
    case 1: return QColor(255, 255, 255);  // 中心实线   白
    case 2: return QColor(255, 160, 0);    // 车道边界   橙
    case 3:                                // 左转箭头
    case 4:                                // 右转箭头
    case 6:                                // 直左箭头
    case 7:                                // 直右箭头
    case 8: return QColor(80, 255, 120);   // 直行箭头   绿
    case 5: return QColor(255, 60, 60);    // 停止线     红
    case 9: return QColor(200, 120, 255);  // 人行横道   紫
    default: return QColor(0, 220, 255);
  }
}

const char* seg_class_name(int cls) {
  // 仅影响 2D 影像包围盒上的显示名称，内部类别索引/逻辑不变
  static const char* kSeg[10] = {"虚线",     "实线",     "实线",
                                 "左转",     "右转",     "停止线",
                                 "直左",     "直右",     "直行",
                                 "人行横道"};
  if (cls < 0 || cls >= 10) return "分割";
  return kSeg[cls];
}

} // namespace

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(320, 180);
  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QColor(12, 14, 18));
  setPalette(pal);
}

void VideoWidget::setImage(const QImage& img) {
  img_ = img;
  update();
}

void VideoWidget::setRawImage(const QImage& img) {
  raw_ = img;
  if (mode_ == ViewMode::Raw) update();
}

void VideoWidget::setFusionImage(const QImage& img) {
  fusion_ = img;
  if (mode_ == ViewMode::Fusion) update();
}

void VideoWidget::setDetections(const std::vector<Detection>& dets) {
  dets_ = dets;
  if (mode_ == ViewMode::Annotated) update();
}

void VideoWidget::setLanes(const std::vector<LanePoly>& lanes) {
  lanes_ = lanes;
  if (mode_ == ViewMode::Annotated) update();
}

void VideoWidget::clear() {
  img_ = QImage();
  raw_ = QImage();
  fusion_ = QImage();
  dets_.clear();
  lanes_.clear();
  danger_text_.clear();
  advice_text_.clear();
  update();
}

void VideoWidget::setOverlayText(const QString& danger, const QString& advice) {
  if (danger == danger_text_ && advice == advice_text_) return;
  danger_text_ = danger;
  advice_text_ = advice;
  update();
}

void VideoWidget::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.fillRect(rect(), QColor(12, 14, 18));

  // 按显示模式选图：识别结果 / 原图 / 双目融合
  const QImage* src = nullptr;
  QString hint = QStringLiteral("等待视频帧…");
  if (mode_ == ViewMode::Fusion) {
    // FPGA 关闭融合图发送时这里始终是空的 → 直接提示无信号
    if (!fusion_.isNull()) src = &fusion_;
    hint = QStringLiteral("无信号");
  } else if (mode_ == ViewMode::Raw) {
    if (!raw_.isNull()) src = &raw_;
    hint = QStringLiteral("等待原始画面…");
  } else {
    if (!img_.isNull()) src = &img_;
  }

  if (!src || src->isNull()) {
    p.setPen(QColor(120, 124, 132));
    QFont f = p.font();
    f.setPixelSize(20);
    p.setFont(f);
    p.drawText(rect(), Qt::AlignCenter, hint);
    return;
  }
  const QImage& shown = *src;
  // 识别结果只在“识别结果”模式下才叠加
  const bool with_overlay = (mode_ == ViewMode::Annotated);

  const double ar_img = static_cast<double>(shown.width()) / shown.height();
  const double ar_box = static_cast<double>(width()) / height();
  QRect target;
  if (ar_box > ar_img) {
    const int w = static_cast<int>(height() * ar_img);
    target = QRect((width() - w) / 2, 0, w, height());
  } else {
    const int h = static_cast<int>(width() / ar_img);
    target = QRect(0, (height() - h) / 2, width(), h);
  }
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  p.drawImage(target, shown);

  // ---- 2D 影像叠加：告警（红）在上，驾驶导引（绿）在下 ----
  // 放在早期 return 之前，保证“显示原图”模式也能看到提示。
  p.setRenderHint(QPainter::Antialiasing, true);
  auto draw_banner = [&](const QString& text, bool danger) {
    if (text.isEmpty()) return;
    QFont f = p.font();
    f.setPixelSize(qMax(16, target.width() / 48));
    f.setBold(true);
    p.setFont(f);
    const QFontMetrics fm(f);
    const int tw = fm.horizontalAdvance(text) + 44;
    const int th = fm.height() + 16;
    const int cy = danger ? (target.top() + 14) : (target.bottom() - th - 14);
    QRect box(target.center().x() - tw / 2, cy, tw, th);
    p.setPen(Qt::NoPen);
    p.setBrush(danger ? QColor(198, 40, 32, 220) : QColor(28, 132, 68, 215));
    p.drawRoundedRect(box, 8, 8);
    p.setPen(QColor(255, 255, 255));
    p.drawText(box, Qt::AlignCenter, text);
  };
  draw_banner(danger_text_, true);
  draw_banner(advice_text_, false);
  p.setRenderHint(QPainter::Antialiasing, false);

  // 叠加只画在主摄画面（识别结果）上
  if (!with_overlay || (dets_.empty() && lanes_.empty())) return;

  const double sx = static_cast<double>(target.width()) / img_.width();
  const double sy = static_cast<double>(target.height()) / img_.height();

  // 车道线：先画框内分割掩码（半透明填充），再画包围盒 + 标签，
  // 与常规 YOLO 分割可视化一致。
  QFont laneFont = p.font();
  laneFont.setPixelSize(12);
  laneFont.setBold(true);
  for (const auto& lp : lanes_) {
    const QColor c = seg_color(lp.cls);

    // 1) 框内掩码填充（line_only 的合并线不画，避免与逐实例框重叠）
    if (!lp.line_only && lp.mw > 0 && lp.mh > 0 &&
        lp.mask.size() == static_cast<size_t>(lp.mw) * lp.mh) {
      QImage ov(lp.mw, lp.mh, QImage::Format_ARGB32);
      ov.fill(Qt::transparent);
      const QRgb fill = qRgba(c.red(), c.green(), c.blue(), 96);
      for (int y = 0; y < lp.mh; ++y) {
        QRgb* line = reinterpret_cast<QRgb*>(ov.scanLine(y));
        const uint8_t* src = lp.mask.data() + static_cast<size_t>(y) * lp.mw;
        for (int x = 0; x < lp.mw; ++x) line[x] = src[x] ? fill : 0u;
      }
      const QRect dst(target.x() + static_cast<int>(lp.bx1 * sx),
                      target.y() + static_cast<int>(lp.by1 * sy),
                      qMax(1, static_cast<int>((lp.bx2 - lp.bx1) * sx)),
                      qMax(1, static_cast<int>((lp.by2 - lp.by1) * sy)));
      p.setRenderHint(QPainter::SmoothPixmapTransform, false);
      p.drawImage(dst, ov);
    }

    // 2) 包围盒
    if (!lp.line_only && lp.bx2 > lp.bx1 && lp.by2 > lp.by1) {
      const QRect r(target.x() + static_cast<int>(lp.bx1 * sx),
                    target.y() + static_cast<int>(lp.by1 * sy),
                    static_cast<int>((lp.bx2 - lp.bx1) * sx),
                    static_cast<int>((lp.by2 - lp.by1) * sy));
      p.setPen(QPen(c, 2));
      p.setBrush(Qt::NoBrush);
      p.drawRect(r);

      p.setFont(laneFont);
      const QString label = QStringLiteral("%1 %2")
                                .arg(QString::fromUtf8(seg_class_name(lp.cls)))
                                .arg(lp.score, 0, 'f', 2);
      const QRect tb(r.x(), qMax(target.y(), r.y() - 16), qMax(80, r.width() / 2 + 40), 16);
      p.fillRect(tb, QColor(c.red(), c.green(), c.blue(), 180));
      p.setPen(QColor(20, 20, 20));
      p.drawText(tb.adjusted(4, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, label);
    }

    // 3) 中心线：黑色外描边 + 本色内芯
    //    实线本色是白色，直接画在浅色路面上肉眼几乎看不见，
    //    所以先描一道深色粗边，再画本色细线，任何背景都清晰。
    if (lp.xy.size() >= 4) {
      QPolygonF poly;
      for (size_t k = 0; k + 1 < lp.xy.size(); k += 2)
        poly << QPointF(target.x() + lp.xy[k] * sx, target.y() + lp.xy[k + 1] * sy);
      const bool dashed = (lp.style == 2);   // LaneStyle::Dashed
      p.setBrush(Qt::NoBrush);

      // 外描边（深色）
      QPen outline(QColor(0, 0, 0, 205), dashed ? 7 : 6);
      outline.setCapStyle(Qt::RoundCap);
      outline.setJoinStyle(Qt::RoundJoin);
      if (dashed) {
        outline.setStyle(Qt::CustomDashLine);
        outline.setDashPattern({2.4, 2.4});   // 单位是线宽
      }
      p.setPen(outline);
      p.drawPolyline(poly);

      // 内芯（本色，实线用高亮白/黄，虚线用青色）
      QColor core = c;
      if (lp.cls == 1) core = QColor(255, 245, 120);   // 实线：亮黄，比纯白更好认
      QPen pen(core, dashed ? 4 : 3);
      pen.setCapStyle(Qt::RoundCap);
      if (dashed) {
        pen.setStyle(Qt::CustomDashLine);
        pen.setDashPattern({4.2, 4.2});
      }
      p.setPen(pen);
      p.drawPolyline(poly);
    }
  }

  p.setRenderHint(QPainter::SmoothPixmapTransform, true);

  QFont f = p.font();
  f.setPixelSize(13);
  f.setBold(true);
  p.setFont(f);

  for (const auto& d : dets_) {
    const QRect r(target.x() + static_cast<int>(d.x1 * sx),
                  target.y() + static_cast<int>(d.y1 * sy),
                  static_cast<int>((d.x2 - d.x1) * sx),
                  static_cast<int>((d.y2 - d.y1) * sy));
    const QColor c = class_color(d.cls);
    p.setPen(QPen(c, 2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(r);

    const QString label =
        QStringLiteral("%1 %2").arg(QString::fromUtf8(det_class_name(d.cls)),
                                    QString::number(d.score, 'f', 2));
    const QRect tb(r.x(), r.y() - 18, 120, 18);
    p.fillRect(tb, QColor(c.red(), c.green(), c.blue(), 180));
    p.setPen(QColor(20, 20, 20));
    p.drawText(tb.adjusted(4, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, label);
  }
}

} // namespace adas
