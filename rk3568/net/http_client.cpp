#include "net/http_client.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace adas {
namespace {

struct Url {
  std::string host;
  int port = 80;
  std::string path = "/";
};

bool parse_url(const std::string& url, Url* out, std::string* err) {
  const std::string prefix = "http://";
  if (url.compare(0, prefix.size(), prefix) != 0) {
    if (err) *err = "only http:// supported: " + url;
    return false;
  }
  std::string rest = url.substr(prefix.size());
  const size_t slash = rest.find('/');
  std::string hostport = (slash == std::string::npos) ? rest : rest.substr(0, slash);
  out->path = (slash == std::string::npos) ? "/" : rest.substr(slash);

  const size_t colon = hostport.rfind(':');
  if (colon != std::string::npos) {
    out->host = hostport.substr(0, colon);
    out->port = std::atoi(hostport.c_str() + colon + 1);
    if (out->port <= 0) out->port = 80;
  } else {
    out->host = hostport;
  }
  if (out->host.empty()) {
    if (err) *err = "empty host";
    return false;
  }
  return true;
}

} // namespace

bool http_post_multipart(const std::string& url, const std::vector<HttpPart>& parts,
                         int timeout_ms, HttpResponse* resp, std::string* err) {
  Url u;
  if (!parse_url(url, &u, err)) return false;

  static const char* kBoundary = "----adasboundary7f3a1c";

  std::string body;
  for (const auto& p : parts) {
    body += "--";
    body += kBoundary;
    body += "\r\n";
    body += "Content-Disposition: form-data; name=\"";
    body += p.name;
    body += "\"";
    if (!p.filename.empty()) {
      body += "; filename=\"";
      body += p.filename;
      body += "\"";
    }
    body += "\r\n";
    std::string ct = p.content_type;
    if (ct.empty()) ct = p.filename.empty() ? "text/plain" : "application/octet-stream";
    body += "Content-Type: ";
    body += ct;
    body += "\r\n\r\n";
    body += p.data;
    body += "\r\n";
  }
  body += "--";
  body += kBoundary;
  body += "--\r\n";

  std::string req;
  req.reserve(body.size() + 256);
  req += "POST ";
  req += u.path;
  req += " HTTP/1.1\r\nHost: ";
  req += u.host;
  if (u.port != 80) {
    req += ":";
    req += std::to_string(u.port);
  }
  req += "\r\nContent-Type: multipart/form-data; boundary=";
  req += kBoundary;
  req += "\r\nContent-Length: ";
  req += std::to_string(body.size());
  req += "\r\nConnection: close\r\nAccept: */*\r\n\r\n";
  req += body;

#ifndef _WIN32
  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* res = nullptr;
  const std::string port = std::to_string(u.port);
  if (getaddrinfo(u.host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
    if (err) *err = "resolve failed: " + u.host;
    return false;
  }

  int fd = -1;
  for (struct addrinfo* ai = res; ai; ai = ai->ai_next) {
    fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (fd < 0) continue;
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) break;
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  if (fd < 0) {
    if (err) *err = "connect failed: " + u.host + ":" + port;
    return false;
  }

  size_t sent = 0;
  while (sent < req.size()) {
    const ssize_t n = ::send(fd, req.data() + sent, req.size() - sent, 0);
    if (n <= 0) {
      ::close(fd);
      if (err) *err = "send failed";
      return false;
    }
    sent += static_cast<size_t>(n);
  }

  std::string raw;
  char buf[4096];
  for (;;) {
    const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n == 0) break;
    if (n < 0) break;   // 超时/错误
    raw.append(buf, static_cast<size_t>(n));
    if (raw.size() > (16u << 20)) break;   // 保护
  }
  ::close(fd);

  const size_t hdr_end = raw.find("\r\n\r\n");
  if (hdr_end == std::string::npos) {
    if (err) *err = "bad response (no header terminator)";
    return false;
  }
  const std::string head = raw.substr(0, hdr_end);
  std::string payload = raw.substr(hdr_end + 4);

  int status = 0;
  if (std::sscanf(head.c_str(), "HTTP/%*d.%*d %d", &status) != 1) {
    if (err) *err = "bad status line";
    return false;
  }

  // 处理 chunked
  std::string lower = head;
  for (auto& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (lower.find("transfer-encoding: chunked") != std::string::npos) {
    std::string decoded;
    size_t pos = 0;
    while (pos < payload.size()) {
      const size_t eol = payload.find("\r\n", pos);
      if (eol == std::string::npos) break;
      const size_t len = std::strtoul(payload.c_str() + pos, nullptr, 16);
      pos = eol + 2;
      if (len == 0) break;
      if (pos + len > payload.size()) { decoded.append(payload, pos, std::string::npos); break; }
      decoded.append(payload, pos, len);
      pos += len + 2;
    }
    payload.swap(decoded);
  }

  if (resp) {
    resp->status = status;
    resp->body = payload;
  }
  if (err) err->clear();
  return true;
#else
  (void)timeout_ms;
  (void)resp;
  (void)req;
  if (err) *err = "http client not implemented on Windows";
  return false;
#endif
}

} // namespace adas
