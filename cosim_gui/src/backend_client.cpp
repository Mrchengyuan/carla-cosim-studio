#include "backend_client.h"

#include <algorithm>
#include <unordered_map>

bool BackendClient::Connect(const std::string& host, int port, std::string& err) {
  Disconnect();
  sock_ = plat::TcpConnect(host, port, err);
  if (sock_ == plat::kInvalidSocket) return false;
  connected_ = true;
  reader_ = std::thread(&BackendClient::ReaderLoop, this, sock_);  // its own copy: Disconnect() resets sock_
  return true;
}

void BackendClient::Disconnect() {
  connected_ = false;
  if (sock_ != plat::kInvalidSocket) {
    plat::CloseSocket(sock_);
    sock_ = plat::kInvalidSocket;
  }
  if (reader_.joinable()) reader_.join();
  {
    // Events the old connection left unread must not reach the next one
    // (telemetry of a dead run, a stale ego or state).
    std::lock_guard<std::mutex> lock(mu_);
    std::deque<json> keep;
    for (auto& m : inbox_)
      if (m.is_object() && m.contains("id")) keep.push_back(std::move(m));
    inbox_.swap(keep);
  }
  FailPending("与后端的连接已断开");
}

void BackendClient::FailPending(const std::string& why) {
  // Every outstanding request gets an error reply, so the UI never waits forever.
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& kv : pending_) inbox_.push_back({{"id", kv.first}, {"ok", false}, {"error", why}});
}

void BackendClient::Request(const std::string& cmd, json args, Callback cb) {
  int id;
  {
    std::lock_guard<std::mutex> lock(mu_);
    id = next_id_++;
    pending_[id] = std::move(cb);
  }
  if (!connected_) {
    std::lock_guard<std::mutex> lock(mu_);
    inbox_.push_back({{"id", id}, {"ok", false}, {"error", "未连接后端"}});
    return;
  }
  json req = {{"id", id}, {"cmd", cmd}, {"args", std::move(args)}};
  // Invalid UTF-8 (e.g. a non-ASCII path from the Windows ANSI code page) is
  // replaced instead of throwing.
  std::string line = req.dump(-1, ' ', false, json::error_handler_t::replace) + "\n";
  std::lock_guard<std::mutex> lock(send_mu_);
  if (!plat::SendAll(sock_, line)) {
    connected_ = false;
    std::lock_guard<std::mutex> lock2(mu_);
    inbox_.push_back({{"id", id}, {"ok", false}, {"error", "发送失败，后端已断开"}});
  }
}

int BackendClient::PendingCount() {
  std::lock_guard<std::mutex> lock(mu_);
  return static_cast<int>(pending_.size());
}

void BackendClient::ReaderLoop(plat::Socket sock) {
  std::string buf;
  size_t searched = 0;  // bytes of buf already known to hold no newline
  int bad_lines = 0;
  char chunk[65536];
  while (connected_) {
    int n = plat::Recv(sock, chunk, sizeof(chunk));
    if (n <= 0) break;
    buf.append(chunk, static_cast<size_t>(n));
    size_t start = 0, pos;
    while ((pos = buf.find('\n', std::max(start, searched))) != std::string::npos) {
      std::string line = buf.substr(start, pos - start);
      start = pos + 1;
      searched = start;
      if (line.empty()) continue;
      json msg = json::parse(line, nullptr, false);
      std::lock_guard<std::mutex> lock(mu_);
      if (msg.is_discarded()) {
        // Should not happen (the backend sends strict JSON); say so rather than
        // silently losing a reply the UI may be waiting for.
        if (bad_lines++ < 5)
          inbox_.push_back({{"event", "log"}, {"level", "error"},
                            {"msg", "收到后端无法解析的消息（" + std::to_string(line.size()) + " 字节），已忽略"}});
        continue;
      }
      inbox_.push_back(std::move(msg));
    }
    buf.erase(0, start);
    searched = buf.size();
  }
  if (connected_) {
    connected_ = false;
    FailPending("与后端的连接已断开");
    std::lock_guard<std::mutex> lock(mu_);
    inbox_.push_back({{"event", "disconnected"}});
  }
}

void BackendClient::Poll(const EventHandler& on_event) {
  std::deque<json> msgs;
  {
    std::lock_guard<std::mutex> lock(mu_);
    msgs.swap(inbox_);
  }
  // Frames of the same view that piled up since the last poll: only the newest
  // is worth decoding and uploading.
  std::unordered_map<std::string, size_t> last_frame;
  for (size_t i = 0; i < msgs.size(); ++i)
    if (msgs[i].is_object() && msgs[i].value("event", std::string()) == "frame")
      last_frame[msgs[i].value("view", std::string("p0"))] = i;
  for (size_t i = 0; i < msgs.size(); ++i) {
    json& msg = msgs[i];
    if (msg.is_object() && msg.value("event", std::string()) == "frame" &&
        last_frame[msg.value("view", std::string("p0"))] != i)
      continue;
    if (msg.contains("id")) {
      Callback cb;
      {
        std::lock_guard<std::mutex> lock(mu_);
        if (!msg["id"].is_number_integer()) continue;
        auto it = pending_.find(msg["id"].get<int>());
        if (it == pending_.end()) continue;
        cb = std::move(it->second);
        pending_.erase(it);
      }
      // A reply or event the UI cannot digest (e.g. an unexpected type) must
      // not take the whole GUI down: report it and go on.
      try {
        if (cb) {
          bool ok = msg.value("ok", false);
          cb(ok, ok ? msg.value("result", json()) : json(), msg.value("error", std::string()));
        }
      } catch (const std::exception& e) {
        if (on_event) on_event({{"event", "log"}, {"level", "error"}, {"msg", std::string("处理后端回复出错：") + e.what()}});
      }
    } else if (on_event) {
      try {
        on_event(msg);
      } catch (const std::exception& e) {
        on_event({{"event", "log"}, {"level", "error"}, {"msg", std::string("处理后端消息出错：") + e.what()}});
      }
    }
  }
}
