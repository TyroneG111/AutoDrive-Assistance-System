#include "event/event_store.h"

#include "common/log.h"
#include "event/jpeg_encoder.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace adas {
namespace {

const char* kIndexName = "index.jsonl";

// 极简 JSON 值提取（索引格式由本模块自行产生，格式固定）
long long json_int(const std::string& line, const std::string& key, long long def) {
  const std::string pat = "\"" + key + "\":";
  const size_t p = line.find(pat);
  if (p == std::string::npos) return def;
  const char* s = line.c_str() + p + pat.size();
  return std::strtoll(s, nullptr, 10);
}

std::string json_str(const std::string& line, const std::string& key) {
  const std::string pat = "\"" + key + "\":\"";
  const size_t p = line.find(pat);
  if (p == std::string::npos) return std::string();
  const size_t b = p + pat.size();
  const size_t e = line.find('"', b);
  if (e == std::string::npos) return std::string();
  return line.substr(b, e - b);
}

} // namespace

EventStore::~EventStore() {
  if (opened_ && dirty_) {
    std::string err;
    save_index(&err);
  }
}

bool EventStore::open(const EventStoreParams& p, std::string* err) {
  p_ = p;
  std::error_code ec;
  fs::create_directories(p_.dir, ec);
  if (ec) {
    if (err) *err = "create_directories failed: " + ec.message();
    return false;
  }
  opened_ = true;
  return load_index(err);
}

std::string EventStore::path_of(const EventItem& it) const {
  return p_.dir + "/" + it.jpeg;
}

bool EventStore::load_index(std::string* err) {
  items_.clear();
  total_ = 0;
  next_id_ = 1;

  const fs::path idx = fs::path(p_.dir) / kIndexName;
  std::ifstream in(idx);
  if (!in) return true;   // 首次运行

  std::string line;
  while (std::getline(in, line)) {
    if (line.size() < 8) continue;
    EventItem it;
    it.id = static_cast<uint64_t>(json_int(line, "id", 0));
    it.ts_ms = json_int(line, "ts", 0);
    it.type = static_cast<int>(json_int(line, "type", 0));
    it.metric = static_cast<float>(json_int(line, "metric_m", 0)) / 1000.0f;
    it.conf = static_cast<float>(json_int(line, "conf_m", 0)) / 1000.0f;
    it.bytes = static_cast<uint32_t>(json_int(line, "bytes", 0));
    it.uploaded = json_int(line, "up", 0) != 0;
    it.jpeg = json_str(line, "f");
    if (it.id == 0 || it.jpeg.empty()) continue;

    // 文件必须存在，否则丢弃该条目
    std::error_code ec;
    if (!fs::exists(path_of(it), ec)) continue;

    if (it.id >= next_id_) next_id_ = it.id + 1;
    total_ += it.bytes;
    items_.push_back(std::move(it));
  }

  std::sort(items_.begin(), items_.end(),
            [](const EventItem& a, const EventItem& b) { return a.id < b.id; });
  if (err) err->clear();
  return true;
}

bool EventStore::reload(std::string* err) { return load_index(err); }

bool EventStore::save_index(std::string* err) {
  const fs::path idx = fs::path(p_.dir) / kIndexName;
  const fs::path tmp = fs::path(p_.dir) / (std::string(kIndexName) + ".tmp");

  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {
    if (err) *err = "open index.tmp failed";
    return false;
  }
  char buf[512];
  for (const auto& it : items_) {
    std::snprintf(buf, sizeof(buf),
                  "{\"id\":%llu,\"ts\":%lld,\"type\":%d,\"metric_m\":%d,\"conf_m\":%d,"
                  "\"bytes\":%u,\"up\":%d,\"f\":\"%s\"}\n",
                  static_cast<unsigned long long>(it.id), static_cast<long long>(it.ts_ms),
                  it.type, static_cast<int>(it.metric * 1000.f + 0.5f),
                  static_cast<int>(it.conf * 1000.f + 0.5f), it.bytes, it.uploaded ? 1 : 0,
                  it.jpeg.c_str());
    out << buf;
  }
  out.close();

  std::error_code ec;
  fs::rename(tmp, idx, ec);
  if (ec) {
    if (err) *err = "rename index failed: " + ec.message();
    return false;
  }
  dirty_ = false;
  return true;
}

void EventStore::enforce_quota() {
  bool removed = false;
  while (total_ > p_.max_total_bytes && !items_.empty()) {
    const EventItem it = items_.front();
    items_.erase(items_.begin());
    std::error_code ec;
    fs::remove(path_of(it), ec);
    total_ = (total_ > it.bytes) ? (total_ - it.bytes) : 0;
    ++evicted_;
    removed = true;
  }
  if (removed) {
    std::string err;
    save_index(&err);
  }
}

bool EventStore::add_rgb(const uint8_t* rgb, int w, int h, int64_t ts_ms, DangerType t,
                         float metric, float conf, uint64_t* id_out, std::string* err) {
  std::string jpeg;
  if (!encode_jpeg(rgb, w, h, p_.jpeg_quality, &jpeg)) {
    if (err) *err = "encode_jpeg failed";
    return false;
  }
  return add_jpeg(jpeg, ts_ms, t, metric, conf, id_out, err);
}

bool EventStore::add_jpeg(const std::string& jpeg, int64_t ts_ms, DangerType t, float metric,
                          float conf, uint64_t* id_out, std::string* err) {
  if (!opened_) {
    if (err) *err = "store not opened";
    return false;
  }
  if (jpeg.empty()) {
    if (err) *err = "empty jpeg";
    return false;
  }

  EventItem it;
  it.id = next_id_++;
  it.ts_ms = ts_ms;
  it.type = static_cast<int>(t);
  it.metric = metric;
  it.conf = conf;
  it.bytes = static_cast<uint32_t>(jpeg.size());
  it.uploaded = false;

  char name[96];
  std::snprintf(name, sizeof(name), "evt_%llu_%lld.jpg",
                static_cast<unsigned long long>(it.id), static_cast<long long>(ts_ms));
  it.jpeg = name;

  {
    std::ofstream f(path_of(it), std::ios::binary | std::ios::trunc);
    if (!f) {
      if (err) *err = "write jpeg failed: " + path_of(it);
      --next_id_;
      return false;
    }
    f.write(jpeg.data(), static_cast<std::streamsize>(jpeg.size()));
    if (!f) {
      if (err) *err = "write jpeg short";
      --next_id_;
      return false;
    }
  }

  total_ += it.bytes;
  items_.push_back(it);
  dirty_ = true;
  save_index(err);
  enforce_quota();

  if (id_out) *id_out = it.id;
  return true;
}

bool EventStore::read_jpeg(const EventItem& it, std::string* out) const {
  if (!out) return false;
  std::ifstream f(path_of(it), std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return !out->empty();
}

bool EventStore::next_pending(EventItem* out) const {
  if (!out) return false;
  for (const auto& it : items_) {
    if (!it.uploaded) {
      *out = it;
      return true;
    }
  }
  return false;
}

bool EventStore::mark_uploaded(uint64_t id) {
  for (auto& it : items_) {
    if (it.id == id) {
      if (it.uploaded) return true;
      it.uploaded = true;
      dirty_ = true;
      std::string err;
      return save_index(&err);
    }
  }
  return false;
}

} // namespace adas
