// The GUI's platform layer (src/platform.cpp) without a window, POSIX: connect
// time-outs, ending the backend process, how an exit is described. Built and
// run by carsim_carla_bridge/tests/test_offline_guimisc.py.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "platform.h"

namespace {

int failures = 0;

void Check(const char* name, bool ok, const std::string& detail = "") {
  std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name, detail.empty() ? "" : "  ", detail.c_str());
  if (!ok) ++failures;
}

double SecondsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

bool WaitGone(const plat::Process& p, double seconds) {
  const auto t0 = std::chrono::steady_clock::now();
  while (plat::IsAlive(p) && SecondsSince(t0) < seconds) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  return !plat::IsAlive(p);
}

}  // namespace

int main() {
  plat::NetInit();
  // A listening socket on a free loopback port, like a backend that is up.
  const int ls = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  a.sin_port = 0;
  socklen_t len = sizeof(a);
  const bool listening = bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0 && listen(ls, 4) == 0 &&
                         getsockname(ls, reinterpret_cast<sockaddr*>(&a), &len) == 0;
  Check("test socket listens", listening);
  const int port = ntohs(a.sin_port);
  std::string err;
  // The quiet retry's short time-out still reaches a backend that listens.
  auto t0 = std::chrono::steady_clock::now();
  plat::Socket s = plat::TcpConnect("127.0.0.1", port, err, 50);
  Check("connect with a 50 ms time-out to a listening port", s != plat::kInvalidSocket, err);
  plat::CloseSocket(s);
  close(ls);
  // Nobody listens now: it fails, within about the time-out.
  err.clear();
  t0 = std::chrono::steady_clock::now();
  s = plat::TcpConnect("127.0.0.1", port, err, 50);
  const double took = SecondsSince(t0);
  Check("refused connect fails with a message", s == plat::kInvalidSocket && !err.empty(), err);
  Check("refused connect returns within the time-out", took < 0.3, std::to_string(took) + " s");
  if (s != plat::kInvalidSocket) plat::CloseSocket(s);

  // DumpStacks signals a live process (SIGUSR1, ignored by this one), nothing without a process.
  plat::Process p;
  err.clear();
  Check("spawn", plat::Spawn({"sh", "-c", "trap '' USR1; exec sleep 30"}, "", "/dev/null", p, err), err);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // (the trap is set)
  Check("DumpStacks signals a live process", plat::DumpStacks(p));
  Check("... which goes on", plat::IsAlive(p));
  // Terminate: SIGTERM without waiting; the exit is described.
  t0 = std::chrono::steady_clock::now();
  plat::Terminate(p);
  Check("Terminate returns at once", SecondsSince(t0) < 0.1);
  Check("terminated process is gone", WaitGone(p, 3.0));
  Check("its exit is described", plat::ExitDescription(p) == "信号 15，被终止", plat::ExitDescription(p));
  plat::Kill(p, true);  // releases it
  Check("released", !p.valid());
  plat::Process none;
  Check("DumpStacks without a process sends nothing", !plat::DumpStacks(none));
  plat::Terminate(none);  // no-op

  // A Python that does not exist: exit code 127 with the hint.
  plat::Process q;
  err.clear();
  Check("spawn a missing program", plat::Spawn({"/nonexistent/python3", "-u"}, "", "/dev/null", q, err), err);
  WaitGone(q, 3.0);
  Check("missing interpreter is explained", plat::ExitDescription(q).find("找不到 Python 解释器") != std::string::npos,
        plat::ExitDescription(q));
  plat::Kill(q, true);

  std::printf(failures ? "%d FAILED\n" : "ALL PLATFORM TESTS PASSED\n", failures);
  return failures ? 1 : 0;
}
