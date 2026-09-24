// Talks to backend_server.py: newline-delimited JSON over TCP.
// Requests are asynchronous; replies and pushed events are queued by a reader
// thread and dispatched on the UI thread in Poll(), so callbacks may touch UI
// state freely.
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "json.hpp"
#include "platform.h"

using json = nlohmann::json;

class BackendClient {
 public:
  using Callback = std::function<void(bool ok, const json& result, const std::string& error)>;
  using EventHandler = std::function<void(const json& event)>;

  ~BackendClient() { Disconnect(); }

  bool Connect(const std::string& host, int port, std::string& err);
  void Disconnect();
  bool Connected() const { return connected_; }

  // Queues a request; cb runs on the UI thread when the reply arrives.
  void Request(const std::string& cmd, json args = json::object(), Callback cb = nullptr);
  int PendingCount();

  // Call once per frame from the UI thread.
  void Poll(const EventHandler& on_event);

 private:
  void ReaderLoop();

  plat::Socket sock_ = plat::kInvalidSocket;
  std::atomic<bool> connected_{false};
  std::thread reader_;
  std::mutex mu_;
  std::deque<json> inbox_;
  std::map<int, Callback> pending_;
  int next_id_ = 1;
  std::mutex send_mu_;
};
