// Qt GUI 入口（与命令行版 adas 分开编译成 adas_gui）
#include "common/log.h"
#include "ui/main_window.h"

#include <QApplication>
#include <QGuiApplication>
#include <QScreen>
#include <QString>
#include <QTimer>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace adas;

namespace {

// SIGTERM/SIGINT -> 置位，由 QTimer 在主线程安全地退出并保存设置
volatile std::sig_atomic_t g_quit_req = 0;
void on_term_signal(int) { g_quit_req = 1; }

void usage() {
  std::printf(
      "usage: adas_gui [--pcap FILE] [--udp PORT] [--bind IP]\n"
      "                [--det DET.rknn] [--seg SEG.rknn]\n"
      "                [--events-dir DIR] [--event-max-mb N]\n"
      "                [--upload http://host:port/path] [--icons DIR]\n"
      "                [--loop] [--no-realtime] [--windowed]\n");
}

} // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName(QStringLiteral("adas_gui"));

  PipelineConfig cfg;
  cfg.realtime = true;
  QString icons;
  bool windowed = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* name) -> std::string {
      (void)name;
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "--pcap") cfg.pcap = next("--pcap");
    else if (a == "--udp") cfg.udp_port = std::stoi(next("--udp"));
    else if (a == "--bind") cfg.bind_ip = next("--bind");
    else if (a == "--det") cfg.det_model = next("--det");
    else if (a == "--seg") cfg.seg_model = next("--seg");
    else if (a == "--events-dir") cfg.events_dir = next("--events-dir");
    else if (a == "--event-max-mb") cfg.event_max_mb = std::stoull(next("--event-max-mb"));
    else if (a == "--upload") cfg.upload_url = next("--upload");
    else if (a == "--icons") icons = QString::fromStdString(next("--icons"));
    else if (a == "--loop") cfg.loop = true;
    else if (a == "--no-realtime") cfg.realtime = false;
    else if (a == "--windowed") windowed = true;
    else if (a == "--help" || a == "-h") { usage(); return 0; }
  }

  MainWindow w(cfg);
  if (!icons.isEmpty()) w.setIconDir(icons);
  if (windowed) {
    w.show();
  } else {
    // 注意：本机 X 虚拟屏是 1920x1280，但物理输出（HDMI-1）只有 1920x1080。
    // 用 primaryScreen()->geometry() 会拿到 1280，窗口被推出物理屏底部，
    // 所以这里仍用 showFullScreen()，由 WM 按实际输出尺寸铺满。
    w.showFullScreen();
  }

  // 退出时统一保存用户设置（关窗口 / Ctrl+C / systemctl stop 都生效）
  QObject::connect(&app, &QCoreApplication::aboutToQuit, &w,
                   [&w]() { w.saveCurrentSettings(); });

  std::signal(SIGINT, on_term_signal);
  std::signal(SIGTERM, on_term_signal);
  QTimer sigTimer;
  QObject::connect(&sigTimer, &QTimer::timeout, [&app]() {
    if (g_quit_req) {
      g_quit_req = 0;
      app.quit();
    }
  });
  sigTimer.start(200);

  const int rc = app.exec();
  w.saveCurrentSettings();   // 双保险
  return rc;
}
