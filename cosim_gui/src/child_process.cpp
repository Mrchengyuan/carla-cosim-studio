#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#endif
#include "child_process.h"

#include <cstring>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#else
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace plat {

namespace {

bool ValidUtf8(const std::string& s) {
  for (size_t i = 0; i < s.size();) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    size_t n;
    if (c < 0x80) n = 0;
    else if ((c >> 5) == 0x6) n = 1;
    else if ((c >> 4) == 0xE) n = 2;
    else if ((c >> 3) == 0x1E) n = 3;
    else return false;
    if (i + n >= s.size() && n) return false;  // cut off
    for (size_t k = 1; k <= n; ++k)
      if ((static_cast<unsigned char>(s[i + k]) & 0xC0) != 0x80) return false;
    i += n + 1;
  }
  return true;
}

#ifdef _WIN32
std::wstring Widen(const std::string& s) {
  if (s.empty()) return L"";
  int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
  std::wstring w(static_cast<size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &w[0], n);
  w.resize(static_cast<size_t>(n - 1));
  return w;
}

std::string Narrow(const std::wstring& w) {
  if (w.empty()) return "";
  int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<size_t>(n), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], n, nullptr, nullptr);
  s.resize(static_cast<size_t>(n - 1));
  return s;
}

// A line in the Windows code page (e.g. GBK from a Chinese Windows tool) -> UTF-8.
std::string FromCodePage(const std::string& s) {
  int n = MultiByteToWideChar(CP_ACP, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
  std::wstring w(static_cast<size_t>(n), L'\0');
  MultiByteToWideChar(CP_ACP, 0, s.data(), static_cast<int>(s.size()), &w[0], n);
  return Narrow(w);
}

BOOL CALLBACK CloseWindowsOf(HWND hwnd, LPARAM pid) {
  DWORD owner = 0;
  GetWindowThreadProcessId(hwnd, &owner);
  if (owner == static_cast<DWORD>(pid) && IsWindowVisible(hwnd)) PostMessageW(hwnd, WM_CLOSE, 0, 0);
  return TRUE;
}
#endif

}  // namespace

std::string QuoteArg(const std::string& a) {
  if (!a.empty() && a.find_first_of(" \t\n\v\"") == std::string::npos) return a;
  std::string out = "\"";
  size_t backslashes = 0;
  for (char c : a) {
    if (c == '\\') {
      ++backslashes;
    } else if (c == '"') {
      out.append(backslashes * 2 + 1, '\\');
      out.push_back('"');
      backslashes = 0;
    } else {
      out.append(backslashes, '\\');
      out.push_back(c);
      backslashes = 0;
    }
  }
  out.append(backslashes * 2, '\\');
  out.push_back('"');
  return out;
}

Child::~Child() {
  Kill();
}

bool Child::Start(const std::vector<std::string>& argv, const std::string& cwd, bool capture, std::string& err) {
  if (argv.empty()) {
    err = "命令为空";
    return false;
  }
  exit_code_ = -1;
#ifdef _WIN32
  std::wstring cmd;
  for (const auto& a : argv) {
    if (!cmd.empty()) cmd += L' ';
    cmd += Widen(QuoteArg(a));
  }
  SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
  HANDLE out_r = nullptr, out_w = nullptr, in_r = nullptr, in_w = nullptr;
  if (capture) {
    if (!CreatePipe(&out_r, &out_w, &sa, 0) || !CreatePipe(&in_r, &in_w, &sa, 0)) {
      err = "CreatePipe 失败，错误码 " + std::to_string(GetLastError());
      return false;
    }
    // Only the child's ends are inherited.
    SetHandleInformation(out_r, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(in_w, HANDLE_FLAG_INHERIT, 0);
  }
  STARTUPINFOW si{};
  si.cb = sizeof(si);
  if (capture) {
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = out_w;
    si.hStdError = out_w;
    si.hStdInput = in_r;
  }
  PROCESS_INFORMATION pi{};
  const std::wstring wcwd = Widen(cwd);
  const DWORD flags = CREATE_SUSPENDED | CREATE_NO_WINDOW | CREATE_UNICODE_ENVIRONMENT;
  const BOOL ok = CreateProcessW(nullptr, &cmd[0], nullptr, nullptr, capture ? TRUE : FALSE, flags, nullptr,
                                 wcwd.empty() ? nullptr : wcwd.c_str(), &si, &pi);
  const DWORD code = GetLastError();
  if (capture) {
    CloseHandle(out_w);
    CloseHandle(in_r);
  }
  if (!ok) {
    if (capture) {
      CloseHandle(out_r);
      CloseHandle(in_w);
    }
    err = code == ERROR_FILE_NOT_FOUND || code == ERROR_PATH_NOT_FOUND
              ? "找不到程序 " + argv[0]
              : "无法启动 " + argv[0] + "（错误码 " + std::to_string(code) + "）";
    return false;
  }
  // Its own job: killed with everything it starts when the job ends (Kill(), or the launcher exits).
  HANDLE job = CreateJobObjectW(nullptr, nullptr);
  if (job) {
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION li{};
    li.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    SetInformationJobObject(job, JobObjectExtendedLimitInformation, &li, sizeof(li));
    if (!AssignProcessToJobObject(job, pi.hProcess)) {
      CloseHandle(job);
      job = nullptr;
    }
  }
  ResumeThread(pi.hThread);
  CloseHandle(pi.hThread);
  process_ = reinterpret_cast<std::intptr_t>(pi.hProcess);
  job_ = reinterpret_cast<std::intptr_t>(job);
  pid_ = static_cast<long>(pi.dwProcessId);
  if (capture) {
    out_ = reinterpret_cast<std::intptr_t>(out_r);
    in_ = reinterpret_cast<std::intptr_t>(in_w);
  }
#else
  int out_p[2] = {-1, -1}, in_p[2] = {-1, -1};
  if (capture && (pipe(out_p) != 0 || pipe(in_p) != 0)) {
    err = "pipe 失败";
    return false;
  }
  // exec failures come back through this pipe (closed on a successful exec).
  int err_p[2];
  if (pipe(err_p) != 0) {
    err = "pipe 失败";
    return false;
  }
  fcntl(err_p[1], F_SETFD, FD_CLOEXEC);
  const pid_t parent = getpid();
  const pid_t pid = fork();
  if (pid < 0) {
    err = "fork 失败";
    return false;
  }
  if (pid == 0) {
    setpgid(0, 0);  // its own group: Kill() ends what it starts too
    prctl(PR_SET_PDEATHSIG, SIGKILL);
    if (getppid() != parent) _exit(127);
    signal(SIGPIPE, SIG_DFL);
    close(err_p[0]);
    if (capture) {
      dup2(in_p[0], 0);
      dup2(out_p[1], 1);
      dup2(out_p[1], 2);
      close(in_p[0]);
      close(in_p[1]);
      close(out_p[0]);
      close(out_p[1]);
    } else {
      int null = open("/dev/null", O_RDWR);
      if (null >= 0) {
        dup2(null, 0);
        dup2(null, 1);
        dup2(null, 2);
        close(null);
      }
    }
    if (!cwd.empty() && chdir(cwd.c_str()) != 0) {
      int e = errno;
      (void)!write(err_p[1], &e, sizeof e);
      _exit(127);
    }
    std::vector<char*> args;
    for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
    args.push_back(nullptr);
    execvp(args[0], args.data());
    int e = errno;
    (void)!write(err_p[1], &e, sizeof e);
    _exit(127);
  }
  setpgid(pid, pid);
  close(err_p[1]);
  int child_errno = 0;
  const ssize_t got = read(err_p[0], &child_errno, sizeof child_errno);
  close(err_p[0]);
  if (capture) {
    close(in_p[0]);
    close(out_p[1]);
    fcntl(in_p[1], F_SETFD, FD_CLOEXEC);
    fcntl(out_p[0], F_SETFD, FD_CLOEXEC);
  }
  if (got == static_cast<ssize_t>(sizeof child_errno)) {
    waitpid(pid, nullptr, 0);
    if (capture) {
      close(in_p[1]);
      close(out_p[0]);
    }
    err = child_errno == ENOENT ? "找不到程序 " + argv[0] : "无法启动 " + argv[0] + "（" + std::strerror(child_errno) + "）";
    return false;
  }
  pid_ = pid;
  if (capture) {
    out_ = out_p[0];
    in_ = in_p[1];
  }
#endif
  started_ = true;
  if (capture) reader_ = std::thread([this] { ReadLoop(); });
  return true;
}

void Child::Push(std::string& partial, const char* data, size_t n, bool flush) {
  std::lock_guard<std::mutex> lock(mu_);
  for (size_t i = 0; i < n; ++i) {
    const char c = data[i];
    if (c == '\n' || c == '\r') {
      if (!partial.empty()) {
#ifdef _WIN32
        lines_.push_back(ValidUtf8(partial) ? partial : FromCodePage(partial));
#else
        lines_.push_back(ValidUtf8(partial) ? partial : std::string("（无法显示的输出）"));
#endif
        partial.clear();
      }
    } else if (partial.size() < 4000) {
      partial.push_back(c);
    }
  }
  if (flush && !partial.empty()) {
    lines_.push_back(partial);
    partial.clear();
  }
  while (lines_.size() > 5000) lines_.pop_front();
}

void Child::ReadLoop() {
  std::string partial;
  char buf[4096];
  for (;;) {
#ifdef _WIN32
    DWORD n = 0;
    if (!ReadFile(reinterpret_cast<HANDLE>(out_), buf, sizeof buf, &n, nullptr) || n == 0) break;
#else
    const ssize_t n = read(static_cast<int>(out_), buf, sizeof buf);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) break;
#endif
    Push(partial, buf, static_cast<size_t>(n), false);
  }
  Push(partial, "", 0, true);
}

bool Child::Running() {
  if (!started_) return false;
  if (exit_code_ != -1) return false;
#ifdef _WIN32
  HANDLE p = reinterpret_cast<HANDLE>(process_);
  if (WaitForSingleObject(p, 0) != WAIT_OBJECT_0) return true;
  DWORD code = 0;
  GetExitCodeProcess(p, &code);
  exit_code_ = static_cast<int>(code == static_cast<DWORD>(-1) ? 255 : code);
#else
  int status = 0;
  const pid_t r = waitpid(static_cast<pid_t>(pid_), &status, WNOHANG);
  if (r == 0) return true;
  if (r < 0) {
    exit_code_ = 255;
  } else {
    exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
  }
#endif
  return false;
}

std::vector<std::string> Child::TakeLines() {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<std::string> out(lines_.begin(), lines_.end());
  lines_.clear();
  return out;
}

void Child::CloseStdin() {
  if (in_ == -1) return;
#ifdef _WIN32
  CloseHandle(reinterpret_cast<HANDLE>(in_));
#else
  close(static_cast<int>(in_));
#endif
  in_ = -1;
}

void Child::RequestClose() {
  if (!started_ || exit_code_ != -1) return;
#ifdef _WIN32
  EnumWindows(CloseWindowsOf, static_cast<LPARAM>(pid_));
#else
  kill(static_cast<pid_t>(pid_), SIGTERM);
#endif
}

void Child::Kill() {
  if (started_) {
#ifdef _WIN32
    if (job_) {
      TerminateJobObject(reinterpret_cast<HANDLE>(job_), 1);
    } else if (exit_code_ == -1) {
      TerminateProcess(reinterpret_cast<HANDLE>(process_), 1);
    }
    WaitForSingleObject(reinterpret_cast<HANDLE>(process_), 3000);
    Running();
#else
    kill(-static_cast<pid_t>(pid_), SIGKILL);
    kill(static_cast<pid_t>(pid_), SIGKILL);
    if (exit_code_ == -1) {
      int status = 0;
      if (waitpid(static_cast<pid_t>(pid_), &status, 0) == static_cast<pid_t>(pid_))
        exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
      else
        exit_code_ = 255;
    }
#endif
  }
  CloseStdin();
  if (reader_.joinable()) reader_.join();  // ends with the pipe (the whole tree is gone)
  if (out_ != -1) {
#ifdef _WIN32
    CloseHandle(reinterpret_cast<HANDLE>(out_));
#else
    close(static_cast<int>(out_));
#endif
    out_ = -1;
  }
#ifdef _WIN32
  if (job_) CloseHandle(reinterpret_cast<HANDLE>(job_));
  if (process_) CloseHandle(reinterpret_cast<HANDLE>(process_));
#endif
  job_ = process_ = 0;
  started_ = false;
}

bool OpenWithSystem(const std::string& target) {
#ifdef _WIN32
  const std::wstring w = Widen(target);
  return reinterpret_cast<std::intptr_t>(ShellExecuteW(nullptr, L"open", w.c_str(), nullptr, nullptr, SW_SHOWNORMAL)) > 32;
#else
  const pid_t pid = fork();
  if (pid == 0) {
    setsid();
    int null = open("/dev/null", O_RDWR);
    if (null >= 0) {
      dup2(null, 0);
      dup2(null, 1);
      dup2(null, 2);
    }
    if (fork() == 0) {  // detached: xdg-open's program is not our child
      execlp("xdg-open", "xdg-open", target.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
    _exit(0);
  }
  if (pid > 0) waitpid(pid, nullptr, 0);
  return pid > 0;
#endif
}

}  // namespace plat
