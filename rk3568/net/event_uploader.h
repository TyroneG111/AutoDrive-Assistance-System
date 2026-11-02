#pragma once
// 事件上传：后台线程把未上传事件以 multipart(JSON + JPEG) POST 到服务端。
#include "event/event_store.h"

#include <atomic>
#include <string>
#include <thread>

namespace adas {

struct UploaderParams {
  std::string url;          // 空 = 不上传
  int timeout_ms = 5000;
  int interval_ms = 2000;   // 空闲轮询间隔
};

class EventUploader {
public:
  ~EventUploader();

  void start(EventStore* store, const UploaderParams& p);
  void stop();

  bool     running() const { return running_.load(); }
  uint64_t uploaded() const { return uploaded_.load(); }
  uint64_t failed() const { return failed_.load(); }

  // 单次上传（同步），供自检与 GUI 手动重传使用
  static bool upload_one(const EventStore& store, const EventItem& it, const std::string& url,
                         int timeout_ms, std::string* err, int* http_status);

private:
  void loop();

  EventStore* store_ = nullptr;
  UploaderParams p_;
  std::thread th_;
  std::atomic<bool> running_{false};
  std::atomic<bool> quit_{false};
  std::atomic<uint64_t> uploaded_{0};
  std::atomic<uint64_t> failed_{0};
};

} // namespace adas
