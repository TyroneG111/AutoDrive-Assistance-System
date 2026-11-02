#include "capture/frame_assembler.h"
#include "capture/pcap_replayer.h"
#include "capture/udp_receiver.h"
#include "common/log.h"
#include "common/types.h"
#include "event/event_store.h"
#include "net/event_uploader.h"
#include "perception/geometry.h"
#include "perception/lane_style.h"
#include "perception/lane_tracker.h"
#include "perception/rules/rule_engine.h"
#include "render/bev_scene.h"
#include "render/soft_raster.h"
#include "vision/mask.h"
#include "vision/postprocess.h"
#include "vision/preprocess.h"
#include "vision/rknn_engine.h"

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

using namespace adas;

namespace {

struct Options {
  std::string pcap;
  int         udp_port = 8080;
  std::string bind_ip = "0.0.0.0";
  std::string det_model;
  std::string seg_model;
  uint64_t    max_packets = 0;
  uint64_t    max_frames = 0;
  int         every = 30;
  int         main_cam = 1;   // 四路拼接时取哪个 1/4（1..4）
  bool        realtime = false;
  bool        selftest = false;
  std::string render_out;
  std::string events_dir;
  uint64_t    event_max_mb = 500;
  std::string upload_url;
  std::string event_selftest_dir;
};

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

std::vector<uint8_t> crop_rgb(const uint8_t* src, int stride_bytes, int x0, int y0, int w, int h) {
  std::vector<uint8_t> out(static_cast<size_t>(w) * h * 3);
  const size_t row = static_cast<size_t>(w) * 3;
  for (int y = 0; y < h; ++y) {
    std::memcpy(out.data() + static_cast<size_t>(y) * row,
                src + static_cast<size_t>(y0 + y) * stride_bytes + static_cast<size_t>(x0) * 3,
                row);
  }
  return out;
}

const std::vector<float>* find_tensor(const std::vector<TensorOut>& v, const char* key) {
  for (const auto& t : v) {
    if (t.name.find(key) != std::string::npos) return &t.data;
  }
  return nullptr;
}

const std::vector<float>* pick(const std::vector<TensorOut>& v, const char* key, size_t idx) {
  const std::vector<float>* p = find_tensor(v, key);
  if (p) return p;
  return (idx < v.size()) ? &v[idx].data : nullptr;
}

// 自检：验证 H 与掩码->折线->BEV 链路（无需 NPU）
void run_selftest() {
  LOGI("=== selftest: H + mask ===");
  const Homography H = make_homography(790.3, 790.3, 640.0, 360.0, 1.3, 2.0);
  const double pts[3][2] = {{640.0, 720.0}, {640.0, 540.0}, {640.0, 450.0}};
  for (const auto& p : pts) {
    double X = 0, Y = 0;
    if (H.project(p[0], p[1], X, Y))
      LOGI("H: px(%.0f,%.0f) -> BEV(%.3f, %.3f) m", p[0], p[1], X, Y);
  }

  std::vector<float> m160(static_cast<size_t>(kProtoSize) * kProtoSize, 0.f);
  for (int y = 0; y < kProtoSize; ++y)
    for (int x = 78; x <= 82; ++x) m160[y * kProtoSize + x] = 0.9f;

  PreprocessMeta meta;
  meta.scale = 0.5f;
  meta.pad_x = 0;
  meta.pad_y = 140;
  meta.src_w = kImgW;
  meta.src_h = kImgH;

  std::vector<uint8_t> bin(static_cast<size_t>(kImgW) * kImgH);
  mask160_to_orig_binary(m160.data(), meta, kImgW, kImgH, bin.data());

  std::vector<float> xs, ys;
  extract_centerline(bin.data(), kImgW, kImgH, 8, 3, xs, ys);
  LOGI("synthetic polyline: %zu pts", xs.size());

  const size_t step = xs.empty() ? 1 : std::max<size_t>(1, xs.size() / 4);
  for (size_t k = 0; k < xs.size(); k += step) {
    if (ys[k] <= 360.f) continue;
    double X = 0, Y = 0;
    if (H.project(xs[k], ys[k], X, Y))
      LOGI("  px(%.0f,%.0f) -> BEV(%.2f, %.2f) m", xs[k], ys[k], X, Y);
  }

  // 线型判定自检
  {
    LaneStyleParams p;
    LaneAnalysis a;
    std::vector<LanePoint> solid;
    for (float X = 2.f; X <= 20.f; X += 0.5f) solid.push_back({X, 0.2f});
    analyze_lane_style(solid, p, &a);
    LOGI("lane(solid): style=%s cont=%.1f segs=%d", lane_style_name(a.style), a.continuous_len,
         a.segments);

    std::vector<LanePoint> dash;
    for (int k = 0; k < 4; ++k) {
      const float base = 2.f + static_cast<float>(k) * 6.f;   // 2m 段 + 4m 间隔
      for (float X = base; X <= base + 2.f; X += 0.5f) dash.push_back({X, 0.2f});
    }
    analyze_lane_style(dash, p, &a);
    LOGI("lane(dashed): style=%s cont=%.1f segs=%d", lane_style_name(a.style), a.continuous_len,
         a.segments);
  }

  // 跟踪 + 本车道自检
  {
    LaneTracker tr;
    std::vector<LaneTrack> tracks;
    for (int f = 0; f < 5; ++f) {
      std::vector<LaneObs> obs;
      LaneObs L;
      L.cls = 2;
      L.style = LaneStyle::Solid;
      for (float X = 2.f; X <= 20.f; X += 1.f) L.pts.push_back({X, 1.75f});
      LaneObs R;
      R.cls = 2;
      R.style = LaneStyle::Dashed;
      for (float X = 2.f; X <= 20.f; X += 1.f) R.pts.push_back({X, -1.75f});
      obs.push_back(std::move(L));
      obs.push_back(std::move(R));
      tr.update(obs, 1.f / 30.f, tracks);
    }
    EgoLane e;
    if (compute_ego_lane(tracks, &e))
      LOGI("ego: width=%.2fm center=%.2fm L=%s R=%s tracks=%zu", e.width, e.center_lat,
           lane_style_name(e.left_style), lane_style_name(e.right_style), tracks.size());
  }

  // 规则引擎自检
  {
    const float dt = 1.f / 30.f;

    // R5 压实线：左侧实线压到足迹带内
    RuleEngine re;
    std::vector<LaneTrack> lanes(1);
    lanes[0].id = 7;
    lanes[0].cls = 2;
    lanes[0].style = LaneStyle::Solid;
    for (int i = 0; i < kGridN; ++i) { lanes[0].lat[i] = 0.85f; lanes[0].ok[i] = true; }
    SceneInput sc;
    sc.dt = dt;
    sc.lanes = &lanes;
    sc.ego.valid = true;
    sc.ego.width = 3.5f;
    sc.ego.center_lat = -0.9f;
    DangerEvent ev;
    for (int i = 0; i < 15; ++i) ev = re.update(sc);
    LOGI("rule(R5): %s area=%.2f m2", danger_name_en(ev.type), ev.metric);

    // R3 闯红灯：红灯 + 停止线逐渐逼近并压线
    RuleEngine re2;
    SceneInput s2;
    s2.dt = dt;
    s2.signal.valid = true;
    s2.signal.red = true;
    s2.signal.score = 0.9f;
    DangerEvent ev2;
    for (int i = 0; i < 40; ++i) {
      s2.stop_lines.clear();
      s2.stop_lines.push_back({0.4f - 0.02f * static_cast<float>(i), 0.9f});
      ev2 = re2.update(s2);
    }
    LOGI("rule(R3): %s dist=%.2f m", danger_name_en(ev2.type), ev2.metric);

    // R7 碰撞预警：前方 5 m 行人
    RuleEngine re3;
    SceneInput s3;
    s3.dt = dt;
    s3.objects.push_back({12, 5.0f, 0.5f, 0.8f});
    DangerEvent ev3;
    for (int i = 0; i < 15; ++i) ev3 = re3.update(s3);
    LOGI("rule(R7): %s dist=%.2f m", danger_name_en(ev3.type), ev3.metric);

    // R4 错道：车道内只有直行箭头，驾驶员在左转
    RuleEngine re4;
    SceneInput s4;
    s4.dt = dt;
    s4.behavior = kBehavLeft;
    s4.ego.valid = true;
    s4.ego.width = 3.5f;
    s4.ego.center_lat = 0.f;
    s4.arrows.push_back({8, 4.0f, 0.2f, 0.9f});
    DangerEvent ev4;
    for (int i = 0; i < 15; ++i) ev4 = re4.update(s4);
    LOGI("rule(R4): %s dlat=%.2f m", danger_name_en(ev4.type), ev4.metric);

    // R2 灯关联：左转行为时应选到左转箭头红灯
    std::vector<LightObs> lights;
    lights.push_back({1, 500.f, 0.8f});   // 红灯(左箭头)
    lights.push_back({3, 640.f, 0.9f});   // 红灯(直行箭头)
    const SignalObs sig = pick_signal(lights, kBehavLeft, 640.f);
    LOGI("rule(R2): valid=%d cls=%d dir=%d red=%d", sig.valid ? 1 : 0, sig.cls, sig.dir,
         sig.red ? 1 : 0);
  }
}

// 自检：BEV 绘制指令 -> 离屏光栅化 -> PPM（无 GUI 环境下检查版面与几何）
void run_render_selftest(const std::string& out) {
  LaneTracker tr;
  std::vector<LaneTrack> tracks;
  for (int f = 0; f < 5; ++f) {
    std::vector<LaneObs> obs;
    LaneObs L;
    L.cls = 1;
    L.style = LaneStyle::Solid;
    for (float X = 2.f; X <= 20.f; X += 1.f) L.pts.push_back({X, 1.75f});
    LaneObs R;
    R.cls = 2;
    R.style = LaneStyle::Dashed;
    for (float X = 2.f; X <= 20.f; X += 1.f) R.pts.push_back({X, -1.75f});
    obs.push_back(std::move(L));
    obs.push_back(std::move(R));
    tr.update(obs, 1.f / 30.f, tracks);
  }

  BevInput in;
  in.lanes = &tracks;
  in.has_ego = true;
  compute_ego_lane(tracks, &in.ego);
  in.objects.push_back({14, 12.0f, -1.0f, 0.90f});   // 轿车
  in.objects.push_back({12, 6.0f, 1.5f, 0.80f});    // 行人
  in.objects.push_back({15, 20.0f, 1.9f, 0.70f});   // 大型车
  in.objects.push_back({13, 9.0f, -2.6f, 0.60f});   // 两三轮
  in.danger = DangerType::CrossSolidLine;
  in.danger_text = "压实线";

  BevParams bp;
  bp.canvas_w = 960;
  bp.canvas_h = 1080;
  BevScene sc;
  build_bev_scene(in, bp, &sc);

  std::vector<uint8_t> rgb;
  if (!rasterize(sc, &rgb)) {
    LOGE("rasterize failed");
    return;
  }
  if (!write_ppm(out.c_str(), rgb.data(), sc.width, sc.height)) {
    LOGE("write %s failed", out.c_str());
    return;
  }
  LOGI("bev: %zu cmds -> %dx%d -> %s", sc.cmds.size(), sc.width, sc.height, out.c_str());
}

// ---------- 事件截图标注 ----------
void class_color(int cls, uint8_t* r, uint8_t* g, uint8_t* b) {
  if (cls < kNumLightClasses) { *r = 255; *g = 214; *b = 0; }
  else if (cls == 12)         { *r = 255; *g = 64;  *b = 64; }
  else if (cls == 13)         { *r = 255; *g = 140; *b = 0;  }
  else                        { *r = 64;  *g = 220; *b = 120; }
}

void draw_box_rgb(uint8_t* rgb, int w, int h, int x0, int y0, int x1, int y1, uint8_t r,
                  uint8_t g, uint8_t b, int th) {
  auto put = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    uint8_t* p = rgb + (static_cast<size_t>(y) * w + x) * 3;
    p[0] = r; p[1] = g; p[2] = b;
  };
  x0 = std::max(0, x0); y0 = std::max(0, y0);
  x1 = std::min(w - 1, x1); y1 = std::min(h - 1, y1);
  for (int t = 0; t < th; ++t) {
    for (int x = x0; x <= x1; ++x) { put(x, y0 + t); put(x, y1 - t); }
    for (int y = y0; y <= y1; ++y) { put(x0 + t, y); put(x1 - t, y); }
  }
}

// 事件首帧截图：在主摄图上叠加检测框
void annotate_frame(std::vector<uint8_t>* img, int w, int h,
                    const std::vector<Detection>& dets) {
  for (const auto& d : dets) {
    uint8_t r = 255, g = 255, b = 255;
    class_color(d.cls, &r, &g, &b);
    draw_box_rgb(img->data(), w, h, static_cast<int>(d.x1), static_cast<int>(d.y1),
                 static_cast<int>(d.x2), static_cast<int>(d.y2), r, g, b, 3);
  }
}

// ---------- 事件记录 + 上传自检 ----------
void run_event_selftest(const std::string& dir, uint64_t max_mb, const std::string& upload_url) {
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);

  const int w = 640, h = 360;
  std::vector<uint8_t> img(static_cast<size_t>(w) * h * 3);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      uint8_t* p = img.data() + (static_cast<size_t>(y) * w + x) * 3;
      p[0] = static_cast<uint8_t>(x * 255 / (w - 1));
      p[1] = static_cast<uint8_t>(y * 255 / (h - 1));
      p[2] = 128;
    }
  std::vector<Detection> fake;
  Detection d;
  d.x1 = 120; d.y1 = 80; d.x2 = 260; d.y2 = 240; d.cls = 14; d.score = 0.9f;
  fake.push_back(d);
  annotate_frame(&img, w, h, fake);

  uint32_t one_bytes = 0;
  {
    EventStoreParams sp1;
    sp1.dir = dir;
    sp1.max_total_bytes = (max_mb ? max_mb : 500) * 1024ull * 1024ull;
    EventStore st1;
    std::string err1;
    if (!st1.open(sp1, &err1)) { LOGE("event store open: %s", err1.c_str()); return; }
    uint64_t id = 0;
    if (!st1.add_rgb(img.data(), w, h, 1700000000000LL, DangerType::CrossSolidLine, 0.62f, 0.91f,
                     &id, &err1)) {
      LOGE("add_rgb failed: %s", err1.c_str());
      return;
    }
    one_bytes = st1.items().empty() ? 0 : st1.items().front().bytes;
    LOGI("event: first jpeg %u bytes, id=%llu", one_bytes, static_cast<unsigned long long>(id));
  }

  // 用小额度重建以验证 FIFO 淘汰（额度 = 2.5 张图）
  EventStoreParams sp;
  sp.dir = dir;
  sp.max_total_bytes = static_cast<uint64_t>(one_bytes) * 5 / 2;
  EventStore st;
  std::string err;
  if (!st.open(sp, &err)) { LOGE("reopen: %s", err.c_str()); return; }
  LOGI("event: quota=%.1f KB, loaded %zu items",
       static_cast<double>(sp.max_total_bytes) / 1024.0, st.count());

  const DangerType types[3] = {DangerType::CollisionWarning, DangerType::WrongLane,
                               DangerType::CrossDashedLine};
  for (int i = 0; i < 3; ++i) {
    uint64_t id = 0;
    if (!st.add_rgb(img.data(), w, h, 1700000001000LL + i, types[i], 1.0f + i, 0.8f, &id,
                    &err)) {
      LOGE("add #%d failed: %s", i, err.c_str());
    }
  }
  LOGI("event: after add -> count=%zu total=%.1f KB evicted=%llu", st.count(),
       static_cast<double>(st.total_bytes()) / 1024.0,
       static_cast<unsigned long long>(st.evicted()));

  EventItem it;
  if (st.next_pending(&it)) {
    LOGI("event: pending id=%llu ts=%lld type=%s bytes=%u",
         static_cast<unsigned long long>(it.id), static_cast<long long>(it.ts_ms),
         danger_name_en(static_cast<DangerType>(it.type)), it.bytes);

    if (!upload_url.empty()) {
      std::string uerr;
      int status = 0;
      if (EventUploader::upload_one(st, it, upload_url, 5000, &uerr, &status)) {
        st.mark_uploaded(it.id);
        LOGI("upload: OK status=%d -> %s", status, upload_url.c_str());
      } else {
        LOGE("upload: FAILED %s (status=%d)", uerr.c_str(), status);
      }
    }
  }

  // 重新加载，验证索引持久化与上传标记
  std::string rerr;
  st.reload(&rerr);
  LOGI("event: reload -> count=%zu total=%.1f KB", st.count(),
       static_cast<double>(st.total_bytes()) / 1024.0);
  for (const auto& e : st.items()) {
    LOGI("  #%llu %s up=%d %.1fKB", static_cast<unsigned long long>(e.id),
         danger_name_en(static_cast<DangerType>(e.type)), e.uploaded ? 1 : 0,
         static_cast<double>(e.bytes) / 1024.0);
  }
}

} // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* n) -> std::string {
      if (i + 1 >= argc) { LOGE("missing value for %s", n); std::exit(2); }
      return argv[++i];
    };
    if (a == "--pcap") opt.pcap = next("--pcap");
    else if (a == "--udp") opt.udp_port = std::stoi(next("--udp"));
    else if (a == "--bind") opt.bind_ip = next("--bind");
    else if (a == "--det") opt.det_model = next("--det");
    else if (a == "--seg") opt.seg_model = next("--seg");
    else if (a == "--max-packets") opt.max_packets = std::stoull(next("--max-packets"));
    else if (a == "--max-frames") opt.max_frames = std::stoull(next("--max-frames"));
    else if (a == "--every") opt.every = std::stoi(next("--every"));
    else if (a == "--realtime") opt.realtime = true;
    else if (a == "--selftest") opt.selftest = true;
    else if (a == "--render-selftest") opt.render_out = next("--render-selftest");
    else if (a == "--events-dir") opt.events_dir = next("--events-dir");
    else if (a == "--event-max-mb") opt.event_max_mb = std::stoull(next("--event-max-mb"));
    else if (a == "--main-cam") opt.main_cam = std::stoi(next("--main-cam"));
    else if (a == "--upload") opt.upload_url = next("--upload");
    else if (a == "--event-selftest") opt.event_selftest_dir = next("--event-selftest");
    else if (a == "--help" || a == "-h") {
      std::printf(
          "usage:\n"
          "  adas --pcap FILE [--realtime] [--det DET.rknn] [--seg SEG.rknn]\n"
          "       [--max-packets N] [--every K] [--events-dir DIR] [--event-max-mb N]\n"
          "       [--upload http://host:port/path]\n"
          "  adas --udp PORT [--bind IP] [--det DET.rknn] [--seg SEG.rknn] [--max-frames N]\n"
          "  adas --selftest\n"
          "  adas --render-selftest OUT.ppm\n"
          "  adas --event-selftest DIR [--event-max-mb N] [--upload URL]\n");
      return 0;
    } else {
      LOGE("unknown arg: %s", a.c_str());
      return 2;
    }
  }

  if (opt.selftest) { run_selftest(); return 0; }
  if (!opt.render_out.empty()) { run_render_selftest(opt.render_out); return 0; }
  if (!opt.event_selftest_dir.empty()) {
    run_event_selftest(opt.event_selftest_dir, opt.event_max_mb, opt.upload_url);
    return 0;
  }

  RknnEngine det_eng, seg_eng;
  bool use_det = false, use_seg = false;
  if (!opt.det_model.empty()) {
    use_det = det_eng.load(opt.det_model);
    if (!use_det) LOGW("det load failed: %s", det_eng.last_error().c_str());
  }
  if (!opt.seg_model.empty()) {
    use_seg = seg_eng.load(opt.seg_model);
    if (!use_seg) LOGW("seg load failed: %s", seg_eng.last_error().c_str());
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::vector<uint8_t> chw(static_cast<size_t>(3) * kModelSize * kModelSize);
  PreprocessMeta meta;
  DetParams dparams;
  LaneStyleParams lsp;
  LaneTracker tracker;
  RuleEngine  rules;

  // 事件存储 + 上传
  EventStore store;
  bool store_ok = false;
  if (!opt.events_dir.empty()) {
    EventStoreParams sp;
    sp.dir = opt.events_dir;
    sp.max_total_bytes = opt.event_max_mb * 1024ull * 1024ull;
    std::string eerr;
    store_ok = store.open(sp, &eerr);
    if (!store_ok) LOGW("event store open failed: %s", eerr.c_str());
    else LOGI("events: dir=%s max=%llu MB", sp.dir.c_str(),
              static_cast<unsigned long long>(opt.event_max_mb));
  }
  EventUploader uploader;
  if (store_ok && !opt.upload_url.empty()) {
    UploaderParams up;
    up.url = opt.upload_url;
    uploader.start(&store, up);
    LOGI("upload: %s", up.url.c_str());
  }
  DangerType last_danger = DangerType::None;
  uint64_t event_saved = 0;

  FrameAssembler assembler;
  uint64_t seen = 0, infer_frames = 0;
  uint64_t det_total = 0, seg_total = 0;

  assembler.set_callback([&](const Frame& f) {
    if (!f.complete) {
      if (seen % 30 == 0)
        LOGW("frame #%llu incomplete (missing %u rows) mode=0x%02X",
             (unsigned long long)f.index, f.missing_rows, f.mode);
      ++seen;
      return;
    }

    // mode=0x01 四路拼接：取主摄对应 1/4；mode=camN 单路：全屏；其余不识别
    const bool sub = (f.mode & kModeFourWay) != 0;
    int cw = 0, ch = 0, x0 = 0, y0 = 0;
    if (sub) {
      const int cam = (opt.main_cam < 1) ? 1 : (opt.main_cam > 4 ? 4 : opt.main_cam);
      cw = 640;
      ch = 360;
      x0 = ((cam - 1) % 2) * 640;
      y0 = ((cam - 1) / 2) * 360;
    } else if (f.mode & 0x1E) {
      cw = kImgW;
      ch = kImgH;
    } else {
      ++seen;
      return;   // 未知模式：不识别
    }

    if (seen % static_cast<uint64_t>(opt.every) == 0) {
      LOGI("frame #%llu complete mode=0x%02X behav=0x%02X fsync=%u ts=%llu",
           (unsigned long long)f.index, f.mode, f.behavior, f.frame_sync,
           (unsigned long long)f.ts_us);
    }
    ++seen;

    if (cw == 0) return;          // 无主摄：不识别
    if (!use_det && !use_seg) return;

    const std::vector<uint8_t> main_rgb = crop_rgb(f.rgb, kRowBytes, x0, y0, cw, ch);
    preprocess(main_rgb.data(), cw, ch, chw.data(), &meta);
    ++infer_frames;

    std::vector<Detection> dets;
    if (use_det && det_eng.run(chw.data())) {
      const auto& outs = det_eng.outputs();
      const auto* boxes = pick(outs, "box", 0);
      const auto* scores = pick(outs, "score", 1);
      if (boxes && scores) {
        dets = decode_nms(boxes->data(), scores->data(), kNumClassesDet,
                          kNumLightClasses, dparams, meta);
        det_total += dets.size();
        if (!dets.empty())
          LOGI("  det: %zu boxes, top score=%.3f cls=%d", dets.size(), dets[0].score, dets[0].cls);
      }
    }

    if (use_seg && seg_eng.run(chw.data())) {
      const auto& outs = seg_eng.outputs();
      const auto* boxes  = pick(outs, "box", 0);
      const auto* scores = pick(outs, "score", 1);
      const auto* coeffs = pick(outs, "coeff", 2);
      const auto* protos = pick(outs, "proto", 3);
      if (boxes && scores) {
        DetParams sp = dparams;
        sp.conf_def = 0.25f;
        const auto segs = decode_nms(boxes->data(), scores->data(), kNumClassesSeg, 0, sp, meta);
        seg_total += segs.size();
        if (!segs.empty())
          LOGI("  seg: %zu instances, top score=%.3f cls=%d", segs.size(), segs[0].score,
               segs[0].cls);

        if (coeffs && protos) {
          const Homography Himg = sub
              ? make_homography(395.2, 395.2, 320.0, 180.0, 1.3, 2.0)
              : make_homography(790.3, 790.3, 640.0, 360.0, 1.3, 2.0);
          const float horizon = sub ? 180.f : 360.f;
          static std::vector<uint8_t> bin;
          static std::vector<float> m160(static_cast<size_t>(kProtoSize) * kProtoSize);
          static std::vector<float> xs, ys;
          bin.resize(static_cast<size_t>(cw) * ch);

          std::vector<LaneObs>     lane_obs;
          std::vector<ArrowObs>    arrows;
          std::vector<StopLineObs> stops;
          lane_obs.reserve(segs.size());

          for (const auto& d : segs) {
            compute_mask160(coeffs->data(), d.mask_idx, protos->data(), m160.data());
            // 必须裁剪到检测框，否则每个实例的掩码都是整张 160x160
            crop_mask160_to_box(m160.data(), d.x1, d.y1, d.x2, d.y2, meta);
            mask160_to_orig_binary(m160.data(), meta, cw, ch, bin.data());

            // 地面箭头 3/4/6/7/8：取掩码质心投影到 BEV
            if (d.cls >= 3 && d.cls <= 8 && d.cls != 5) {
              float u = 0.f, v = 0.f;
              if (!mask_centroid(bin.data(), cw, ch, 0, &u, &v)) continue;
              if (v <= horizon) continue;
              double X = 0, Y = 0;
              if (Himg.project(u, v, X, Y) && X > 0.0)
                arrows.push_back({d.cls, static_cast<float>(X), static_cast<float>(Y), d.score});
              continue;
            }
            // 停止线 5：取最靠本车一侧的像元，得到车头到线的纵向距离
            if (d.cls == 5) {
              float u = 0.f, v = 0.f;
              if (!mask_centroid(bin.data(), cw, ch, 4, &u, &v)) continue;
              if (v <= horizon) continue;
              double X = 0, Y = 0;
              if (Himg.project(u, v, X, Y) && X > 0.0)
                stops.push_back({static_cast<float>(X), d.score});
              continue;
            }
            if (d.cls > 2) continue;   // 0/1/2 = 中心虚线/中心实线/车道边界线

            extract_centerline(bin.data(), cw, ch, 8, 3, xs, ys);
            if (xs.empty()) continue;

            std::vector<LanePoint> lp;
            lp.reserve(xs.size());
            for (size_t k = 0; k < xs.size(); ++k) {
              if (ys[k] <= horizon) continue;   // 地平线以上不投影
              double X = 0, Y = 0;
              if (Himg.project(xs[k], ys[k], X, Y) && X > 0.0)
                lp.push_back({static_cast<float>(X), static_cast<float>(Y)});
            }
            if (lp.size() < 3) continue;

            LaneAnalysis la;
            const bool has = analyze_lane_style(lp, lsp, &la);
            LaneObs ob;
            ob.cls = d.cls;
            ob.style = has ? la.style : LaneStyle::Unknown;
            ob.pts = lp;
            lane_obs.push_back(std::move(ob));

            if (has)
              LOGI("  seg[cls=%d] score=%.2f style=%s cont=%.1fm segs=%d lat=%.2fm", d.cls,
                   d.score, lane_style_name(la.style), la.continuous_len, la.segments,
                   la.mean_lateral);
          }

          std::vector<LaneTrack> tracks;
          tracker.update(lane_obs, 1.f / 30.f, tracks);

          // 检测目标 -> BEV 地面点（底边中点），信号灯 -> 图像中心 x
          std::vector<ObjectObs> objs;
          std::vector<LightObs>  lights;
          objs.reserve(dets.size());
          for (const auto& d : dets) {
            if (d.cls < kNumLightClasses) {
              lights.push_back({d.cls, 0.5f * (d.x1 + d.x2), d.score});
              continue;
            }
            double X = 0, Y = 0;
            if (!Himg.project(0.5 * (d.x1 + d.x2), d.y2, X, Y) || X <= 0.0) continue;
            objs.push_back({d.cls, static_cast<float>(X), static_cast<float>(Y), d.score});
          }

          SceneInput sc;
          sc.dt = 1.f / 30.f;
          sc.behavior = f.behavior;
          sc.image_mode = f.mode;
          sc.has_main_camera = true;
          sc.lanes = &tracks;
          sc.arrows = std::move(arrows);
          sc.stop_lines = std::move(stops);
          sc.objects = std::move(objs);
          sc.signal = pick_signal(lights, f.behavior, sub ? 320.f : 640.f);

          if (compute_ego_lane(tracks, &sc.ego)) {
            LOGI("  ego-lane: L#%d(%s) R#%d(%s) width=%.2fm center=%.2fm tracks=%zu",
                 sc.ego.left_id, lane_style_name(sc.ego.left_style), sc.ego.right_id,
                 lane_style_name(sc.ego.right_style), sc.ego.width, sc.ego.center_lat,
                 tracks.size());
          }

          const DangerEvent ev = rules.update(sc);
          if (ev.type != DangerType::None) {
            LOGI("  DANGER: %s metric=%.2f conf=%.2f track=%d active=%zu",
                 danger_name_en(ev.type), ev.metric, ev.conf, ev.track_id,
                 rules.active().size());

            // 事件截图：只存首帧
            if (store_ok && last_danger == DangerType::None) {
              std::vector<uint8_t> shot = main_rgb;
              annotate_frame(&shot, cw, ch, dets);
              const int64_t ts_ms = f.ts_us ? static_cast<int64_t>(f.ts_us / 1000)
                                            : wall_ms();
              uint64_t eid = 0;
              std::string eerr;
              if (store.add_rgb(shot.data(), cw, ch, ts_ms, ev.type, ev.metric, ev.conf, &eid,
                                &eerr)) {
                ++event_saved;
                LOGI("  event: #%llu %s saved", static_cast<unsigned long long>(eid),
                     danger_name_en(ev.type));
              } else {
                LOGW("  event save failed: %s", eerr.c_str());
              }
            }
            last_danger = ev.type;
          } else {
            last_danger = DangerType::None;
          }
        }
      }
    }
  });

  const double t0 = now_ms();

  if (!opt.pcap.empty()) {
    PcapReplayer rp;
    if (!rp.open(opt.pcap)) {
      LOGE("open pcap failed: %s", opt.pcap.c_str());
      return 1;
    }
    LOGI("pcap: %s (pcapng=%d)", opt.pcap.c_str(), rp.is_pcapng() ? 1 : 0);
    rp.replay([&](const uint8_t* p, size_t n, uint64_t ts) { assembler.push(p, n, ts); },
              opt.max_packets, opt.realtime);
    assembler.flush();
  } else {
    UdpReceiver rx;
    if (!rx.open(static_cast<uint16_t>(opt.udp_port), opt.bind_ip)) {
      LOGE("bind udp %s:%d failed", opt.bind_ip.c_str(), opt.udp_port);
      return 1;
    }
    LOGI("udp listening on %s:%d", opt.bind_ip.c_str(), opt.udp_port);
    std::vector<uint8_t> buf(2048);
    while (!g_stop) {
      const int n = rx.recv(buf.data(), buf.size(), nullptr, 200);
      if (n > 0) assembler.push(buf.data(), static_cast<size_t>(n), 0);
      else if (n < 0) break;
      if (opt.max_frames && assembler.frames_out() >= opt.max_frames) break;
    }
    assembler.flush();
  }

  const double dt = (now_ms() - t0) / 1000.0;
  uploader.stop();
  LOGI("done: packets=%llu frames=%llu incomplete=%llu infer=%llu det=%.0f seg=%.0f",
       (unsigned long long)assembler.packets(), (unsigned long long)assembler.frames_out(),
       (unsigned long long)assembler.frames_incomplete(), (unsigned long long)infer_frames,
       static_cast<double>(det_total), static_cast<double>(seg_total));
  if (store_ok)
    LOGI("events: saved=%llu stored=%zu total=%.1f KB evicted=%llu uploaded=%llu",
         (unsigned long long)event_saved, store.count(),
         static_cast<double>(store.total_bytes()) / 1024.0,
         static_cast<unsigned long long>(store.evicted()),
         static_cast<unsigned long long>(uploader.uploaded()));
  if (dt > 0)
    LOGI("elapsed %.2f s, %.1f packets/s", dt, assembler.packets() / dt);
  return 0;
}
