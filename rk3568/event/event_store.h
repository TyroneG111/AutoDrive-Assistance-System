#pragma once
// 事件记录：JPEG 截图 + 索引，总容量上限内 FIFO 淘汰最旧事件。
// 索引用 JSONL（每行一条），无需 SQLite 依赖。
#include "perception/rules/rule_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace adas {

struct EventItem {
  uint64_t id = 0;
  int64_t  ts_ms = 0;
  int      type = 0;      // DangerType
  float    metric = 0.f;
  float    conf = 0.f;
  uint32_t bytes = 0;
  bool     uploaded = false;
  std::string jpeg;       // 文件名（相对 dir）
};

struct EventStoreParams {
  std::string dir = "events";
  uint64_t max_total_bytes = 500ull * 1024 * 1024;   // 500 MB
  int jpeg_quality = 80;
};

class EventStore {
public:
  ~EventStore();

  bool open(const EventStoreParams& p, std::string* err);

  // 由 RGB 编码为 JPEG 并落盘
  bool add_rgb(const uint8_t* rgb, int w, int h, int64_t ts_ms, DangerType t, float metric,
               float conf, uint64_t* id_out, std::string* err);

  // 已有 JPEG 数据落盘
  bool add_jpeg(const std::string& jpeg, int64_t ts_ms, DangerType t, float metric, float conf,
                uint64_t* id_out, std::string* err);

  bool read_jpeg(const EventItem& it, std::string* out) const;

  const std::vector<EventItem>& items() const { return items_; }
  uint64_t total_bytes() const { return total_; }
  size_t   count() const { return items_.size(); }
  uint64_t evicted() const { return evicted_; }
  const std::string& dir() const { return p_.dir; }

  // 上传队列：最早一条未上传事件
  bool next_pending(EventItem* out) const;
  bool mark_uploaded(uint64_t id);

  // 重新从磁盘加载索引（用于 GUI 刷新/重启恢复）
  bool reload(std::string* err);

private:
  bool load_index(std::string* err);
  bool save_index(std::string* err);
  void enforce_quota();
  std::string path_of(const EventItem& it) const;

  EventStoreParams p_;
  std::vector<EventItem> items_;
  uint64_t total_ = 0;
  uint64_t evicted_ = 0;
  uint64_t next_id_ = 1;
  bool dirty_ = false;
  bool opened_ = false;
};

} // namespace adas
