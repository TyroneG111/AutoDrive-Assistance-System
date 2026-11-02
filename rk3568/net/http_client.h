#pragma once
// 极简 HTTP/1.1 客户端（仅 http://，multipart/form-data POST），无第三方依赖。
#include <string>
#include <vector>

namespace adas {

struct HttpPart {
  std::string name;
  std::string filename;      // 空表示普通字段
  std::string content_type;  // 空则按 filename 推断
  std::string data;
};

struct HttpResponse {
  int status = 0;
  std::string body;
  std::string content_type;
};

// url: http://host[:port]/path
bool http_post_multipart(const std::string& url, const std::vector<HttpPart>& parts,
                         int timeout_ms, HttpResponse* resp, std::string* err);

} // namespace adas
