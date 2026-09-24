#include "backend_client.h"

bool BackendClient::Connect(const std::string& host, int port, std::string& err) {
  Disconnect();
  sock_ = plat::TcpConnect(host, port, err);
  if (sock_ == plat::kInvalidSocket) return false;
  connected_ = true;
  reader_ = std::thread(&BackendClient::ReaderLoop, this);
  return true;
}

void BackendClient::Disconnect() {
  connected_ = false;
  if (sock_ != plat::kInvalidSocket) {
    plat::CloseSocket(sock_);
    sock_ = plat::kInvalidSocket;
  }
  if (reader_.joinable()) reader_.join();
  std::lock_guard<std::mutex> lock(mu_);
  // Fail every outstanding request so the UI never waits forever.
  for (auto& kv : pending_) {
    json fail = {{"id", kv.first}, {"ok", false}, {"error", "与后端的连接已断开"}};
    inbox_.push_back(fail);
  }
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
  std::string line = req.dump() + "\n";
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

void BackendClient::ReaderLoop() {
  std::string buf;
  char chunk[65536];
  while (connected_) {
    int n = plat::Recv(sock_, chunk, sizeof(chunk));
    if (n <= 0) break;
    buf.append(chunk, static_cast<size_t>(n));
    size_t pos;
    while ((pos = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, pos);
      buf.erase(0, pos + 1);
      if (line.empty()) continue;
      json msg = json::parse(line, nullptr, false);
      if (msg.is_discarded()) continue;
      std::lock_guard<std::mutex> lock(mu_);
      inbox_.push_back(std::move(msg));
    }
  }
  if (connected_) {
    connected_ = false;
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
  for (auto& msg : msgs) {
    if (msg.contains("id")) {
      Callback cb;
      {
        std::lock_guard<std::mutex> lock(mu_);
        auto it = pending_.find(msg["id"].get<int>());
        if (it == pending_.end()) continue;
        cb = std::move(it->second);
        pending_.erase(it);
      }
      if (cb) {
        bool ok = msg.value("ok", false);
        cb(ok, ok ? msg.value("result", json()) : json(), msg.value("error", std::string()));
      }
    } else if (on_event) {
      on_event(msg);
    }
  }
}
