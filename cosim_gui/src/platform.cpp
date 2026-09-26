#include "platform.h"

#include <filesystem>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <shellapi.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace plat {

bool NetInit() {
#ifdef _WIN32
  WSADATA wsa;
  return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
#else
  signal(SIGPIPE, SIG_IGN);  // a closed backend must not kill the GUI
  return true;
#endif
}

Socket TcpConnect(const std::string& host, int port, std::string& err) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  const std::string port_str = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || !res) {
    err = "无法解析地址 " + host;
    return kInvalidSocket;
  }
  Socket s = static_cast<Socket>(socket(res->ai_family, res->ai_socktype, res->ai_protocol));
  if (s == kInvalidSocket) {
    freeaddrinfo(res);
    err = "socket() 失败";
    return kInvalidSocket;
  }
  // Connect with a time-out: this runs on the UI thread, and on Windows a
  // refused loopback connect otherwise blocks for about two seconds.
  const auto fd = static_cast<decltype(socket(0, 0, 0))>(s);
#ifdef _WIN32
  u_long nb = 1;
  ioctlsocket(fd, FIONBIO, &nb);
#else
  const int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
  bool ok = connect(fd, res->ai_addr, static_cast<int>(res->ai_addrlen)) == 0;
  freeaddrinfo(res);
  if (!ok) {
    fd_set wr, ex;
    FD_ZERO(&wr);
    FD_ZERO(&ex);
    FD_SET(fd, &wr);
    FD_SET(fd, &ex);
    timeval tv{0, 300000};
    if (select(static_cast<int>(fd + 1), nullptr, &wr, &ex, &tv) > 0 && FD_ISSET(fd, &wr)) {
      int so_err = 0;
      socklen_t len = sizeof(so_err);
      ok = getsockopt(fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_err), &len) == 0 && so_err == 0;
    }
  }
#ifdef _WIN32
  nb = 0;
  ioctlsocket(fd, FIONBIO, &nb);
#else
  fcntl(fd, F_SETFL, flags);
#endif
  if (!ok) {
    CloseSocket(s);
    err = "连接 " + host + ":" + port_str + " 失败（后端未启动？）";
    return kInvalidSocket;
  }
  int one = 1;
  setsockopt(static_cast<decltype(socket(0, 0, 0))>(s), IPPROTO_TCP, TCP_NODELAY,
             reinterpret_cast<const char*>(&one), sizeof(one));
  return s;
}

bool SendAll(Socket s, const std::string& data) {
  size_t sent = 0;
  while (sent < data.size()) {
    const int n = static_cast<int>(send(static_cast<decltype(socket(0, 0, 0))>(s), data.data() + sent,
                                        static_cast<int>(data.size() - sent), 0));
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}

int Recv(Socket s, char* buf, int len) {
  return static_cast<int>(recv(static_cast<decltype(socket(0, 0, 0))>(s), buf, len, 0));
}

void CloseSocket(Socket s) {
  if (s == kInvalidSocket) return;
#ifdef _WIN32
  shutdown(static_cast<SOCKET>(s), SD_BOTH);
  closesocket(static_cast<SOCKET>(s));
#else
  shutdown(static_cast<int>(s), SHUT_RDWR);
  close(static_cast<int>(s));
#endif
}

#ifdef _WIN32
static std::wstring Widen(const std::string& s) {
  if (s.empty()) return L"";
  int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
  std::wstring w(static_cast<size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &w[0], n);
  w.resize(static_cast<size_t>(n - 1));
  return w;
}
#endif

bool Spawn(const std::vector<std::string>& argv, const std::string& cwd,
           const std::string& log_path, Process& out, std::string& err) {
  if (argv.empty()) {
    err = "命令为空";
    return false;
  }
#ifdef _WIN32
  std::wstring cmd;
  for (const auto& a : argv) {
    if (!cmd.empty()) cmd += L' ';
    cmd += L'"' + Widen(a) + L'"';
  }
  SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
  HANDLE log = CreateFileW(Widen(log_path).c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                           &sa, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  STARTUPINFOW si{};
  si.cb = sizeof(si);
  si.dwFlags = STARTF_USESTDHANDLES;
  si.hStdOutput = log;
  si.hStdError = log;
  si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  PROCESS_INFORMATION pi{};
  std::wstring wcwd = Widen(cwd);
  BOOL ok = CreateProcessW(nullptr, &cmd[0], nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                           wcwd.empty() ? nullptr : wcwd.c_str(), &si, &pi);
  if (log != INVALID_HANDLE_VALUE) CloseHandle(log);
  if (!ok) {
    err = "CreateProcess 失败，错误码 " + std::to_string(GetLastError());
    return false;
  }
  CloseHandle(pi.hThread);
  out.handle = reinterpret_cast<std::intptr_t>(pi.hProcess);
  out.status = -1;
  return true;
#else
  pid_t pid = fork();
  if (pid < 0) {
    err = "fork 失败";
    return false;
  }
  if (pid == 0) {
    if (!cwd.empty() && chdir(cwd.c_str()) != 0) _exit(127);
    int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
      dup2(fd, 1);
      dup2(fd, 2);
      close(fd);
    }
    std::vector<char*> args;
    for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
    args.push_back(nullptr);
    execvp(args[0], args.data());
    _exit(127);
  }
  out.handle = pid;
  out.status = -1;
  return true;
#endif
}

bool IsAlive(const Process& p) {
  if (!p.valid()) return false;
#ifdef _WIN32
  DWORD code = 0;
  if (!GetExitCodeProcess(reinterpret_cast<HANDLE>(p.handle), &code)) return false;
  if (code == STILL_ACTIVE) return true;
  p.status = static_cast<int>(code);
  return false;
#else
  if (p.status != -1) return false;  // already reaped
  int status = 0;
  const pid_t r = waitpid(static_cast<pid_t>(p.handle), &status, WNOHANG);
  if (r == 0) return true;
  if (r > 0) p.status = status;
  return false;
#endif
}

void Kill(Process& p, bool hard) {
  if (!p.valid()) return;
#ifdef _WIN32
  // No SIGTERM on Windows: the backend was asked to shut down (it cleans CARLA
  // up and exits); give it the time, then terminate it and wait until it is
  // gone, so its log file is free again.
  HANDLE h = reinterpret_cast<HANDLE>(p.handle);
  if (!hard) WaitForSingleObject(h, 5000);
  if (WaitForSingleObject(h, 0) == WAIT_TIMEOUT) {
    TerminateProcess(h, 1);
    WaitForSingleObject(h, 5000);
  }
  CloseHandle(h);
#else
  const pid_t pid = static_cast<pid_t>(p.handle);
  if (IsAlive(p)) {
    if (!hard) {
      kill(pid, SIGTERM);
      for (int i = 0; i < 50 && IsAlive(p); ++i) usleep(100000);
    }
    if (IsAlive(p)) {
      kill(pid, SIGKILL);
      int status = 0;
      if (waitpid(pid, &status, 0) > 0) p.status = status;
    }
  }
#endif
  p.handle = 0;
  p.status = -1;
}

std::string ExitDescription(const Process& p) {
#ifdef _WIN32
  return p.status == -1 ? std::string("原因未知") : "退出码 " + std::to_string(static_cast<unsigned>(p.status));
#else
  if (p.status == -1) return "原因未知";
  if (WIFEXITED(p.status))
    return WEXITSTATUS(p.status) == 127 ? std::string("退出码 127，找不到 Python 解释器，请在“连接”页设置")
                                        : "退出码 " + std::to_string(WEXITSTATUS(p.status));
  if (WIFSIGNALED(p.status)) {
    const int sig = WTERMSIG(p.status);
    const char* what = sig == SIGSEGV ? "段错误" : sig == SIGABRT ? "程序中止" : sig == SIGKILL ? "被强制结束"
                     : sig == SIGTERM ? "被终止" : sig == SIGBUS ? "总线错误" : sig == SIGFPE ? "算术错误" : "";
    return "信号 " + std::to_string(sig) + (*what ? std::string("，") + what : std::string());
  }
  return "原因未知";
#endif
}

void DumpStacks(const Process& p) {
#ifndef _WIN32
  if (IsAlive(p)) kill(static_cast<pid_t>(p.handle), SIGUSR1);
#else
  (void)p;
#endif
}

std::vector<std::string> Utf8Args(int argc, char** argv) {
  std::vector<std::string> out;
#ifdef _WIN32
  int n = 0;
  LPWSTR* w = CommandLineToArgvW(GetCommandLineW(), &n);
  if (w) {
    for (int i = 0; i < n; ++i) {
      int len = WideCharToMultiByte(CP_UTF8, 0, w[i], -1, nullptr, 0, nullptr, nullptr);
      std::string s(static_cast<size_t>(len > 0 ? len - 1 : 0), '\0');
      if (len > 1) WideCharToMultiByte(CP_UTF8, 0, w[i], -1, &s[0], len, nullptr, nullptr);
      out.push_back(s);
    }
    LocalFree(w);
    return out;
  }
#endif
  for (int i = 0; i < argc; ++i) out.emplace_back(argv[i]);
  return out;
}

std::string ExecutableDir() {
#ifdef _WIN32
  wchar_t buf[MAX_PATH];
  GetModuleFileNameW(nullptr, buf, MAX_PATH);
  return std::filesystem::path(buf).parent_path().u8string();
#else
  std::error_code ec;
  auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
  return ec ? std::string(".") : p.parent_path().string();
#endif
}

bool FileExists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::exists(std::filesystem::u8path(path), ec);
}

}  // namespace plat
