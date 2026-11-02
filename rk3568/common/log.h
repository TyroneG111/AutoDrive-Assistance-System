#pragma once
#include <chrono>
#include <cstdio>

namespace adas {
inline double now_ms() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

// 墙钟毫秒（Unix epoch），用于事件时间戳；不受系统开机时长影响
inline int64_t wall_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
} // namespace adas

#define LOGI(...) do { std::fprintf(stdout, "[I] "); std::fprintf(stdout, __VA_ARGS__); std::fputc('\n', stdout); } while (0)
#define LOGW(...) do { std::fprintf(stderr, "[W] "); std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); } while (0)
#define LOGE(...) do { std::fprintf(stderr, "[E] "); std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); } while (0)
