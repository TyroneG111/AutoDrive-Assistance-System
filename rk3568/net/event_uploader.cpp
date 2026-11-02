#include "net/event_uploader.h"

#include "common/log.h"
#include "net/http_client.h"

#include <chrono>
#include <cstdio>

namespace adas {
namespace {

std::string build_meta_json(const EventItem& it) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "{\"id\":%llu,\"ts_ms\":%lld,\"type\":\"%s\",\"type_id\":%d,"
                "\"metric\":%.3f,\"conf\":%.3f,\"bytes\":%u}",
                static_cast<unsigned long long>(it.id), static_cast<long long>(it.ts_ms),
                danger_name(static_cast<DangerType>(it.type)), it.type,
                static_cast<double>(it.metric), static_cast<double>(it.conf), it.bytes);
  return std::string(buf);
}

} // namespace

EventUploader::~EventUploader() { stop(); }

bool EventUploader::upload_one(const EventStore& store, const EventItem& it,
                               const std::string& url, int timeout_ms, std::string* err,
                               int* http_status) {
  std::string jpeg;
  if (!store.read_jpeg(it, &jpeg)) {
    if (err) *err = "read jpeg failed";
    return false;
  }

  std::vector<HttpPart> parts;
  HttpPart meta;
  meta.name = "meta";
  meta.content_type = "application/json";
  meta.data = build_meta_json(it);
  parts.push_back(meta);

  HttpPart img;
  img.name = "image";
  img.filename = it.jpeg;
  img.content_type = "image/jpeg";
  img.data.swap(jpeg);
  parts.push_back(std::move(img));

  HttpResponse resp;
  if (!http_post_multipart(url, parts, timeout_ms, &resp, err)) return false;
  if (http_status) *http_status = resp.status;
  if (resp.status < 200 || resp.status >= 300) {
    if (err) *err = "http status " + std::to_string(resp.status);
    return false;
  }
  return true;
}

void EventUploader::start(EventStore* store, const UploaderParams& p) {
  if (running_.load() || !store || p.url.empty()) return;
  store_ = store;
  p_ = p;
  quit_.store(false);
  running_.store(true);
  th_ = std::thread(&EventUploader::loop, this);
}

void EventUploader::stop() {
  if (!running_.load()) return;
  quit_.store(true);
  if (th_.joinable()) th_.join();
  running_.store(false);
}

void EventUploader::loop() {
  while (!quit_.load()) {
    EventItem it;
    bool did = false;
    if (store_ && store_->next_pending(&it)) {
      std::string err;
      int status = 0;
      if (upload_one(*store_, it, p_.url, p_.timeout_ms, &err, &status)) {
        store_->mark_uploaded(it.id);
        uploaded_.fetch_add(1);
        LOGI("upload ok: id=%llu type=%s status=%d", static_cast<unsigned long long>(it.id),
             danger_name_en(static_cast<DangerType>(it.type)), status);
      } else {
        failed_.fetch_add(1);
        LOGW("upload failed: id=%llu err=%s", static_cast<unsigned long long>(it.id),
             err.c_str());
        // 失败后等待一个周期，避免打爆服务端
        std::this_thread::sleep_for(std::chrono::milliseconds(p_.interval_ms));
      }
      did = true;
    }
    if (!did || quit_.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(p_.interval_ms));
  }
}

} // namespace adas
