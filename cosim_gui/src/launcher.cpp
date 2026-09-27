#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#endif
#include "launcher.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "imgui.h"
#include "imgui_internal.h"
#include "json.hpp"
#include "platform.h"
#include "ui_kit.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <aclapi.h>
#include <iphlpapi.h>
#include <sddl.h>
#else
#include <csignal>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include <GLFW/glfw3.h>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

constexpr const char* kDocUrl =
    "https://github.com/Mrchengyuan/carla-cosim-studio/blob/main/docs/%E8%BF%9C%E7%A8%8B%E4%BD%BF%E7%94%A8%E6%8C%87%E5%8D%97.md";
constexpr const char* kPythonUrl = "https://www.python.org/downloads/windows/";
// Python version, pointer size, program, then numpy's version (a separate line:
// without numpy the first line still comes).
constexpr const char* kPyProbe =
    "import sys,struct;print('COSIM_PY',sys.version.split()[0],struct.calcsize('P')*8,sys.executable,flush=True);"
    "import numpy;print('COSIM_NUMPY',numpy.__version__,flush=True)";
constexpr double kConnectTimeout = 90.0;   // s: ssh up to "connected"
constexpr double kReadyTimeout = 420.0;    // s: connected to "就绪" (CARLA's first start is about 1 min)
constexpr int kMaxReconnects = 5;

enum Check { kFiles, kSsh, kPython, kNumpy, kPorts };

std::string U8(const fs::path& p) { return p.u8string(); }

bool StartsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }
bool Contains(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

// fopen with a UTF-8 path (on Windows fopen takes the ANSI code page: a Chinese user name breaks it).
FILE* OpenFile(const std::string& path, const char* mode) {
#ifdef _WIN32
  std::wstring wp, wm;
  const int n = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
  wp.resize(static_cast<size_t>(n > 0 ? n - 1 : 0));
  if (n > 1) MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, &wp[0], n);
  for (const char* c = mode; *c; ++c) wm.push_back(static_cast<wchar_t>(*c));
  return _wfopen(wp.c_str(), wm.c_str());
#else
  return std::fopen(path.c_str(), mode);
#endif
}

std::string NowText() {
  std::time_t t = std::time(nullptr);
  char buf[16];
  std::strftime(buf, sizeof buf, "%H:%M:%S", std::localtime(&t));
  return buf;
}

std::string Elapsed(double s) {
  const int n = static_cast<int>(s);
  char buf[32];
  if (n < 60) std::snprintf(buf, sizeof buf, "%d 秒", n);
  else if (n < 3600) std::snprintf(buf, sizeof buf, "%d 分 %02d 秒", n / 60, n % 60);
  else std::snprintf(buf, sizeof buf, "%d 小时 %02d 分", n / 3600, (n / 60) % 60);
  return buf;
}

#ifdef _WIN32
std::wstring W(const std::string& s) {
  if (s.empty()) return L"";
  int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
  std::wstring w(static_cast<size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &w[0], n);
  w.resize(static_cast<size_t>(n - 1));
  return w;
}
std::string N(const wchar_t* w) {
  int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<size_t>(n > 0 ? n - 1 : 0), '\0');
  if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
  return s;
}
std::string SearchExe(const wchar_t* name) {
  wchar_t buf[MAX_PATH];
  const DWORD n = SearchPathW(nullptr, name, nullptr, MAX_PATH, buf, nullptr);
  return n > 0 && n < MAX_PATH ? N(buf) : "";
}
std::string EnvDir(const wchar_t* var) {
  wchar_t buf[MAX_PATH];
  const DWORD n = GetEnvironmentVariableW(var, buf, MAX_PATH);
  return n > 0 && n < MAX_PATH ? N(buf) : "";
}
// Wine keeps no Windows permissions on its Linux files (it derives them from the
// mode bits, and setting a DACL makes the file 0777): the key is left as it is there.
bool UnderWine() {
  HMODULE nt = GetModuleHandleW(L"ntdll.dll");
  return nt && GetProcAddress(nt, "wine_get_version") != nullptr;
}
#else
std::string SearchExe(const char* name) {
  const char* path = std::getenv("PATH");
  std::stringstream ss(path ? path : "/usr/bin:/bin");
  std::string d;
  while (std::getline(ss, d, ':')) {
    const std::string p = (d.empty() ? "." : d) + "/" + name;
    if (access(p.c_str(), X_OK) == 0) return p;
  }
  return "";
}
#endif

// What an ssh / server line means for the user (empty: nothing to add).
// *fatal: it stays so until someone changes something (the key, the port, the
// server): no reconnecting; otherwise (the network) reconnecting may help.
std::string SshHint(const std::string& l, bool* fatal) {
  *fatal = true;
  if (Contains(l, "Permission denied"))
    return "云服务器没有接受启动包里的钥匙：钥匙和服务器不匹配（服务器可能重装或换过）。请向管理员要一个新的启动包。";
  if (Contains(l, "Host key verification failed") || Contains(l, "REMOTE HOST IDENTIFICATION HAS CHANGED") ||
      Contains(l, "No ED25519 host key is known") || Contains(l, "host key is known"))
    return "服务器的身份和启动包里记录的不一致（服务器可能重装或换过）。为了安全没有连接，请向管理员要一个新的启动包。";
  if (Contains(l, "UNPROTECTED PRIVATE KEY") || Contains(l, "bad permissions"))
    return "钥匙文件的权限太宽，SSH 拒绝使用。把启动包解压到自己的文件夹（例如桌面或“文档”）里再运行。";
  if (Contains(l, "Address already in use") || Contains(l, "cannot listen to port") ||
      Contains(l, "Could not request local forwarding"))
    return "这台电脑上的端口 57120 / 57121 被占用：可能还有上一次的连接没关。关掉其他启动器窗口后点“重新检查”；还不行就重启电脑。";
  *fatal = false;
  if (Contains(l, "Could not resolve hostname") || Contains(l, "Name or service not known") ||
      Contains(l, "No such host"))
    return "找不到服务器地址：检查这台电脑能否上网（例如打开一个网页试试）。";
  if (Contains(l, "timed out") || Contains(l, "Operation timed out"))
    return "连不上服务器（超时）：检查网络；云服务器可能已关机或已到期。";
  if (Contains(l, "Connection refused"))
    return "服务器拒绝连接：云服务器可能刚重启、SSH 还没起来，或端口变了。稍等一会儿再试；一直不行请联系管理员。";
  if (Contains(l, "Network is unreachable") || Contains(l, "No route to host"))
    return "网络不通：检查这台电脑的网络连接。";
  if (Contains(l, "not responding") || Contains(l, "Broken pipe"))
    return "云服务器没有响应（网络中断了一会儿？）。";
  if (Contains(l, "closed by") || Contains(l, "Connection reset"))
    return "连接被中断了（网络问题，或云服务器重启了）。";
  return "";
}

}  // namespace

// ============================================================================
// lifecycle
// ============================================================================
Launcher::Launcher() { t0_ = phase_t_ = start_t_ = Clock::now(); }

Launcher::~Launcher() {
  // The job objects / process groups end everything anyway; do it in order.
  if (gui_proc_ && gui_proc_->Running()) {
    gui_proc_->RequestClose();
    for (int i = 0; i < 30 && gui_proc_->Running(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (service_proc_) service_proc_->Kill();
  if (ssh_) {
    ssh_->CloseStdin();
    for (int i = 0; i < 30 && ssh_->Running(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ssh_->Kill();
  }
  if (gui_proc_) gui_proc_->Kill();
  if (probe_) probe_->Kill();
  if (pip_) pip_->Kill();
  if (!tour_dir_.empty()) WriteState();
}

void Launcher::Init(const std::vector<std::string>& args) {
  plat::NetInit();
  dir_ = plat::ExecutableDir();
  const fs::path d = fs::u8path(dir_);
  ssh_dir_ = U8(d / "ssh");
  key_ = U8(d / "ssh" / "remote_key");
  known_ = U8(d / "ssh" / "known_hosts");
  service_ = U8(d / "service" / "carsim_service.py");
#ifdef _WIN32
  gui_exe_ = U8(d / "carla_cosim_studio.exe");
  fs::path st = fs::u8path(EnvDir(L"LOCALAPPDATA").empty() ? dir_ : EnvDir(L"LOCALAPPDATA")) / "carla_cosim_studio";
  // The children print UTF-8 (the CarSim service reads these too).
  SetEnvironmentVariableW(L"PYTHONIOENCODING", L"utf-8");
  SetEnvironmentVariableW(L"PYTHONUNBUFFERED", L"1");
#else
  gui_exe_ = U8(d / "carla_cosim_studio");
  const char* xdg = std::getenv("XDG_CACHE_HOME");
  const char* home = std::getenv("HOME");
  fs::path st = (xdg && *xdg ? fs::path(xdg) : fs::path(home ? home : "/tmp") / ".cache") / "carla_cosim_studio";
  setenv("PYTHONIOENCODING", "utf-8", 1);
  setenv("PYTHONUNBUFFERED", "1", 1);
#endif
  settings_path_ = U8(d / "remote_launcher.json");
  notes_ = U8(d / u8"使用说明.txt");
  std::error_code ec;
  fs::create_directories(st, ec);
  state_dir_ = U8(st);
  log_path_ = U8(st / "remote_launch.log");
  fs::rename(fs::u8path(log_path_), fs::u8path(log_path_ + ".prev"), ec);

  std::string script = "main";
  for (size_t i = 1; i < args.size(); ++i) {
    const std::string& a = args[i];
    auto next = [&](std::string& v) {
      if (i + 1 < args.size()) v = args[++i];
    };
    std::string v;
    if (a == "--tour") next(tour_dir_);
    else if (a == "--tour-script") next(script);
    else if (a == "--gui-arg") { next(v); gui_extra_.push_back(v); }
    else if (a == "--no-auto-start") auto_start_ = false;
    else if (a == "--local-ports") {  // tests on the server itself: its own ports are taken
      next(v);
      std::sscanf(v.c_str(), "%d,%d", &local_gui_port_, &local_carsim_port_);
    }
  }
  LoadSettings();
  checks_.resize(5);
  checks_[kFiles].name = "启动包文件";
  checks_[kSsh].name = "SSH 客户端";
  checks_[kPython].name = "Python";
  checks_[kNumpy].name = "numpy";
  checks_[kPorts].name = "本机端口";
  Log(kLauncher, kInfo, "启动器目录：" + dir_);
  if (!tour_dir_.empty()) BuildTour(script);
  StartChecks();
}

void Launcher::LoadSettings() {
  json j;
  std::ifstream f(fs::u8path(settings_path_));
  if (f) {
    try {
      f >> j;
    } catch (const std::exception&) {
      Log(kLauncher, kWarn, "remote_launcher.json 读不出来，已忽略");
      j = json::object();
    }
  }
  if (!j.is_object()) j = json::object();
  host_ = j.value("host", std::string());
  user_ = j.value("user", std::string());
  port_ = j.value("port", 0);
  python_setting_ = j.value("python", std::string());
  dark_ = j.value("dark_theme", true);
}

void Launcher::SaveSettings() {
  json j;
  std::ifstream in(fs::u8path(settings_path_));
  if (in) {
    try {
      in >> j;
    } catch (const std::exception&) {
      j = json::object();
    }
  }
  if (!j.is_object()) j = json::object();
  j["python"] = python_setting_;
  j["dark_theme"] = dark_;
  std::ofstream out(fs::u8path(settings_path_));
  if (out) out << j.dump(2) << "\n";
  else Log(kLauncher, kWarn, "设置没能保存（" + settings_path_ + " 不可写）");
}

bool Launcher::ThemeChanged() {
  const bool c = theme_changed_;
  theme_changed_ = false;
  return c;
}

int Launcher::TakeWindowRequest() {
  const int r = window_request_;
  window_request_ = 0;
  return r;
}

std::string Launcher::WindowTitle() const {
  switch (phase_) {
    case Phase::Ready: return "远程仿真 · 就绪";
    case Phase::Connecting:
    case Phase::Starting: return "远程仿真 · 正在启动";
    case Phase::Failed: return "远程仿真 · 出错";
    default: return "远程仿真启动器";
  }
}

bool Launcher::Animating() const {
  return phase_ == Phase::Checking || phase_ == Phase::Connecting || phase_ == Phase::Starting ||
         phase_ == Phase::Stopping || pip_ != nullptr || reconnect_pending_ || !tour_dir_.empty() ||
         (phase_ == Phase::Ready && gui_.st == St::Busy);
}

bool Launcher::Running() const {
  return phase_ == Phase::Connecting || phase_ == Phase::Starting || phase_ == Phase::Ready ||
         phase_ == Phase::Stopping;
}

double Launcher::Since(Clock::time_point t) const {
  return std::chrono::duration<double>(Clock::now() - t).count();
}

void Launcher::SetPhase(Phase p) {
  phase_ = p;
  phase_t_ = Clock::now();
}

void Launcher::Log(int src, int lvl, const std::string& text) {
  log_.push_back({NowText(), src, lvl, text});
  if (log_.size() > 4000) log_.erase(log_.begin(), log_.begin() + 1000);
  static const char* kSrc[] = {"启动器", "云端", "CarSim", "界面"};
  static const char* kLvl[] = {"", "", "[警告] ", "[错误] "};
  if (FILE* f = OpenFile(log_path_, "ab")) {
    std::fprintf(f, "%s %s  %s%s\n", log_.back().time.c_str(), kSrc[src], kLvl[lvl], text.c_str());
    std::fclose(f);
  }
}

// ============================================================================
// environment checks
// ============================================================================
void Launcher::StartChecks() {
  SetPhase(Phase::Checking);
  checks_ok_ = false;
  for (auto& c : checks_) {
    c.st = St::Busy;
    c.detail.clear();
    c.hint.clear();
  }
  Log(kLauncher, kInfo, "检查这台电脑的环境…");
  CheckFiles();
  CheckSsh();
  CheckPorts();
  StartPythonProbe();
}

void Launcher::CheckFiles() {
  Check& c = checks_[kFiles];
  std::vector<std::string> missing;
  for (const auto& p : {key_, known_, service_, gui_exe_})
    if (!plat::FileExists(p)) missing.push_back(U8(fs::relative(fs::u8path(p), fs::u8path(dir_))));
  if (host_.empty() || user_.empty() || port_ <= 0) missing.push_back("remote_launcher.json（服务器地址）");
  if (missing.empty()) {
    c.st = St::Ok;
    c.detail = "完整";
    return;
  }
  c.st = St::Fail;
  std::string list;
  for (const auto& m : missing) list += (list.empty() ? "" : "、") + m;
  c.detail = "缺少 " + list;
  c.hint = "启动包没有完整解压：把整个压缩包解压（右键 → 全部解压缩），再运行解压出来的“启动远程仿真”。";
}

void Launcher::CheckSsh() {
  Check& c = checks_[kSsh];
#ifdef _WIN32
  std::vector<std::string> cands;
  const std::string root = EnvDir(L"SystemRoot");
  if (!root.empty()) {
    cands.push_back(root + "\\System32\\OpenSSH\\ssh.exe");
    cands.push_back(root + "\\Sysnative\\OpenSSH\\ssh.exe");
  }
  cands.push_back(SearchExe(L"ssh.exe"));
  ssh_exe_.clear();
  for (const auto& p : cands)
    if (!p.empty() && plat::FileExists(p)) {
      ssh_exe_ = p;
      break;
    }
#else
  ssh_exe_ = SearchExe("ssh");
#endif
  if (!ssh_exe_.empty()) {
    c.st = St::Ok;
    c.detail = ssh_exe_;
  } else {
    c.st = St::Fail;
    c.detail = "没有找到 ssh.exe";
    c.hint = "Windows 自带的“OpenSSH 客户端”没有安装：打开 设置 → 应用 → 可选功能 → 添加功能，安装“OpenSSH 客户端”，然后点“重新检查”。";
  }
}

std::string Launcher::PortOwner(int port, long* pid) {
  *pid = 0;
#ifdef _WIN32
  ULONG size = 0;
  GetExtendedTcpTable(nullptr, &size, FALSE, AF_INET, TCP_TABLE_OWNER_PID_LISTENER, 0);
  std::vector<unsigned char> buf(size + 1024);
  size = static_cast<ULONG>(buf.size());
  if (GetExtendedTcpTable(buf.data(), &size, FALSE, AF_INET, TCP_TABLE_OWNER_PID_LISTENER, 0) != NO_ERROR) return "";
  auto* t = reinterpret_cast<MIB_TCPTABLE_OWNER_PID*>(buf.data());
  for (DWORD i = 0; i < t->dwNumEntries; ++i) {
    if (ntohs(static_cast<u_short>(t->table[i].dwLocalPort)) != port) continue;
    *pid = static_cast<long>(t->table[i].dwOwningPid);
    if (*pid <= 0) return "";  // unknown (Wine does not say)
    std::string name = "PID " + std::to_string(*pid);
    if (HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, t->table[i].dwOwningPid)) {
      wchar_t path[MAX_PATH];
      DWORD n = MAX_PATH;
      if (QueryFullProcessImageNameW(h, 0, path, &n)) name = U8(fs::path(path).filename());
      CloseHandle(h);
    }
    return name;
  }
  return "";
#else
  const std::string cmd = "ss -ltnpH 'sport = :" + std::to_string(port) + "' 2>/dev/null";
  FILE* f = popen(cmd.c_str(), "r");
  if (!f) return "";
  char line[1024];
  std::string out;
  while (std::fgets(line, sizeof line, f)) out += line;
  pclose(f);
  const size_t q = out.find("((\"");
  if (q == std::string::npos) return out.empty() ? "" : "其他程序";
  const size_t e = out.find('"', q + 3);
  const size_t pp = out.find("pid=", q);
  if (pp != std::string::npos) *pid = std::atol(out.c_str() + pp + 4);
  return out.substr(q + 3, e - q - 3);
#endif
}

void Launcher::CheckPorts() {
  Check& c = checks_[kPorts];
  busy_pids_.clear();
  std::string busy;
  for (int port : {local_gui_port_, local_carsim_port_}) {
    std::string err;
    plat::Socket s = plat::TcpConnect("127.0.0.1", port, err, 200);
    if (s == plat::kInvalidSocket) continue;
    plat::CloseSocket(s);
    long pid = 0;
    const std::string owner = PortOwner(port, &pid);
    busy += (busy.empty() ? "" : "，") + std::to_string(port) + (owner.empty() ? "" : "（" + owner + "）");
    std::string low = owner;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    if (pid > 0 && (low == "ssh" || low == "ssh.exe")) busy_pids_.push_back(pid);
  }
  const std::string ports = std::to_string(local_gui_port_) + " / " + std::to_string(local_carsim_port_);
  if (busy.empty()) {
    c.st = St::Ok;
    c.detail = ports + " 空闲";
  } else {
    c.st = St::Fail;
    c.detail = "被占用：" + busy;
    c.hint = busy_pids_.empty()
                 ? "有其他程序占用了连接云端要用的端口。关掉它（或重启电脑）后点“重新检查”。"
                 : "上一次的云端连接（ssh）还开着，占用了这些端口。点“结束旧连接”即可。";
  }
}

void Launcher::StartPythonProbe() {
  py_candidates_.clear();
  py_try_ = 0;
  py_argv_.clear();
  py_version_ = py_bits_ = py_exe_ = numpy_version_ = "";
  checks_[kPython].st = checks_[kNumpy].st = St::Busy;
  checks_[kPython].detail = "正在查找…";
  checks_[kNumpy].detail = "";
  if (!python_setting_.empty()) {
    py_candidates_.push_back({python_setting_});
  } else {
#ifdef _WIN32
    const std::string py = SearchExe(L"py.exe");
    if (!py.empty()) py_candidates_.push_back({py, "-3"});
    const std::string python = SearchExe(L"python.exe");
    if (!python.empty()) py_candidates_.push_back({python});
#else
    for (const char* n : {"python3", "python"}) {
      const std::string p = SearchExe(n);
      if (!p.empty()) py_candidates_.push_back({p});
    }
#endif
  }
  probe_.reset();
  if (py_candidates_.empty()) return;  // UpdateChecks reports it
  probe_out_.clear();
  probe_ = std::make_unique<plat::Child>();
  std::vector<std::string> argv = py_candidates_[0];
  argv.push_back("-c");
  argv.push_back(kPyProbe);
  std::string err;
  probe_t_ = Clock::now();
  if (!probe_->Start(argv, dir_, true, err)) {
    Log(kLauncher, kWarn, err);
    probe_.reset();
    py_try_ = 1;
  }
}

void Launcher::UpdateChecks() {
  if (probe_) {
    for (auto& l : probe_->TakeLines()) probe_out_.push_back(l);
    const bool running = probe_->Running();
    if (running && Since(probe_t_) < 30.0) return;
    if (running) {
      Log(kLauncher, kWarn, "Python 30 秒内没有响应：" + py_candidates_[py_try_][0]);
      probe_->Kill();
    }
    for (auto& l : probe_->TakeLines()) probe_out_.push_back(l);
    for (const auto& l : probe_out_) {
      std::istringstream ss(l);
      std::string tag;
      ss >> tag;
      if (tag == "COSIM_PY") {
        ss >> py_version_ >> py_bits_;
        std::getline(ss, py_exe_);
        py_exe_.erase(0, py_exe_.find_first_not_of(' '));
        py_argv_ = py_candidates_[py_try_];
      } else if (tag == "COSIM_NUMPY") {
        ss >> numpy_version_;
      }
    }
    probe_.reset();
    if (py_argv_.empty()) {
      // This one does not run (e.g. the Microsoft Store placeholder "python.exe"): the next.
      std::string why = probe_out_.empty() ? "" : "：" + probe_out_.back();
      Log(kLauncher, kInfo, "不能用的 Python：" + py_candidates_[py_try_][0] + why);
      ++py_try_;
      if (py_try_ < py_candidates_.size()) {
        probe_out_.clear();
        probe_ = std::make_unique<plat::Child>();
        std::vector<std::string> argv = py_candidates_[py_try_];
        argv.push_back("-c");
        argv.push_back(kPyProbe);
        std::string err;
        probe_t_ = Clock::now();
        if (!probe_->Start(argv, dir_, true, err)) probe_.reset();
        return;
      }
    }
  }
  FinishChecks();
}

void Launcher::FinishChecks() {
  Check& py = checks_[kPython];
  Check& np = checks_[kNumpy];
  if (py_argv_.empty()) {
    py.st = St::Fail;
    np.st = St::Idle;
    np.detail = "（需要先有 Python）";
    if (!python_setting_.empty()) {
      py.detail = "指定的 Python 不能运行：" + python_setting_;
      py.hint = "在右边“Python”里重新选择 python.exe，或改回“自动查找”。";
    } else {
      py.detail = "没有找到 Python";
      py.hint = "需要 64 位 Python 3.8 或更新版本（CarSim 服务用它）。从 python.org 下载安装，安装时勾选"
                "“Add python.exe to PATH”，装好后点“重新检查”；已经装了的，在右边“Python”里选择它的 python.exe。";
    }
  } else {
    const std::string cmd = py_argv_.size() > 1 ? "py -3" : py_exe_;
    py.detail = "Python " + py_version_ + "（" + py_bits_ + " 位）  " + py_exe_;
    if (py_bits_ != "64") {
      py.st = St::Fail;
      py.hint = "这是 32 位 Python，CarSim 的求解器是 64 位的：请安装 64 位 Python，或在右边选择 64 位的 python.exe。";
    } else {
      py.st = St::Ok;
    }
    if (numpy_version_.empty()) {
      np.st = St::Fail;
      np.detail = "没有安装";
      np.hint = "CarSim 服务需要 numpy：点“安装 numpy”自动安装（需要联网，约 1 分钟）。";
    } else {
      np.st = St::Ok;
      np.detail = "numpy " + numpy_version_;
    }
  }
  checks_ok_ = std::all_of(checks_.begin(), checks_.end(), [](const Check& c) { return c.st == St::Ok; });
  if (checks_ok_) {
    Log(kLauncher, kOk, "环境检查通过（" + checks_[kPython].detail + "，" + np.detail + "）");
  } else {
    for (const auto& c : checks_)
      if (c.st == St::Fail) Log(kLauncher, kErr, c.name + "：" + c.detail);
  }
  SetPhase(Phase::Idle);
  if (checks_ok_ && auto_start_) Start();
  auto_start_ = false;
}

void Launcher::PrepareKey() {
  // OpenSSH refuses a key others can read. The package's folder may not allow
  // that (an exFAT USB stick has no permissions at all, a folder others share):
  // the key and known_hosts go to this user's own folder (%LOCALAPPDATA% on NTFS),
  // copied again at every start (a new package brings a new key).
  ssh_cwd_ = ssh_dir_;
#ifdef _WIN32
  if (UnderWine()) return;
#endif
  const fs::path d = fs::u8path(state_dir_) / "ssh";
  std::error_code ec;
  fs::create_directories(d, ec);
  for (const char* f : {"remote_key", "known_hosts"}) {
    fs::remove(d / f, ec);
    if (!fs::copy_file(fs::u8path(ssh_dir_) / f, d / f, ec)) {
      Log(kLauncher, kWarn, "没能把钥匙复制到 " + U8(d) + "（" + ec.message() + "），直接用启动包里的");
      FixKey(key_);
      return;
    }
  }
  ssh_cwd_ = U8(d);
  FixKey(U8(d / "remote_key"));
}

void Launcher::FixKey(const std::string& key) {
  // Only this user may read it: a protected DACL with one entry (like icacls /inheritance:r /grant:r).
#ifdef _WIN32
  HANDLE tok = nullptr;
  if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &tok)) return;
  DWORD n = 0;
  GetTokenInformation(tok, TokenUser, nullptr, 0, &n);
  std::vector<unsigned char> buf(n);
  bool ok = false;
  if (GetTokenInformation(tok, TokenUser, buf.data(), n, &n)) {
    PSID sid = reinterpret_cast<TOKEN_USER*>(buf.data())->User.Sid;
    EXPLICIT_ACCESSW ea{};
    ea.grfAccessPermissions = GENERIC_READ | GENERIC_WRITE | DELETE | READ_CONTROL | WRITE_DAC;
    ea.grfAccessMode = SET_ACCESS;
    ea.grfInheritance = NO_INHERITANCE;
    ea.Trustee.TrusteeForm = TRUSTEE_IS_SID;
    ea.Trustee.TrusteeType = TRUSTEE_IS_USER;
    ea.Trustee.ptstrName = reinterpret_cast<LPWSTR>(sid);
    PACL acl = nullptr;
    if (SetEntriesInAclW(1, &ea, nullptr, &acl) == ERROR_SUCCESS) {
      std::wstring path = W(key);
      ok = SetNamedSecurityInfoW(&path[0], SE_FILE_OBJECT,
                                 DACL_SECURITY_INFORMATION | PROTECTED_DACL_SECURITY_INFORMATION, nullptr, nullptr,
                                 acl, nullptr) == ERROR_SUCCESS;
      LocalFree(acl);
    }
  }
  CloseHandle(tok);
  if (!ok) Log(kLauncher, kWarn, "没能收紧钥匙文件的权限；如果连接时提示 bad permissions，把启动包解压到自己的文件夹里再运行");
#else
  if (chmod(key.c_str(), 0600) != 0) Log(kLauncher, kWarn, "chmod 600 失败：" + key);
#endif
}

void Launcher::InstallNumpy() {
  if (py_argv_.empty() || pip_) return;
  std::vector<std::string> argv = py_argv_;
  for (const char* a : {"-m", "pip", "install", "--disable-pip-version-check", "-i",
                        "https://pypi.tuna.tsinghua.edu.cn/simple", "numpy"})
    argv.push_back(a);
  pip_ = std::make_unique<plat::Child>();
  std::string err;
  if (!pip_->Start(argv, dir_, true, err)) {
    Log(kLauncher, kErr, "没能运行 pip：" + err);
    pip_.reset();
    return;
  }
  checks_[kNumpy].st = St::Busy;
  checks_[kNumpy].detail = "正在安装…";
  Log(kLauncher, kInfo, "安装 numpy（清华镜像）…");
}

// ============================================================================
// start / stop
// ============================================================================
void Launcher::Start() {
  if (Running()) return;
  CheckPorts();  // an old tunnel may have come up since the check
  if (checks_[kPorts].st != St::Ok) {
    Log(kLauncher, kErr, "本机端口" + checks_[kPorts].detail);
    SetPhase(Phase::Idle);
    return;
  }
  fail_title_ = fail_hint_ = stop_note_ = ssh_error_ = server_name_ = "";
  cloud_ = {St::Busy, "正在连接…"};
  svc_ = {St::Idle, "等待云端连接"};
  gui_ = {St::Idle, "等待云端就绪"};
  tunnel_up_ = server_ready_ = ready_once_ = reconnect_pending_ = service_started_ = false;
  reconnects_ = 0;
  start_t_ = Clock::now();
  PrepareKey();
  Log(kLauncher, kInfo, "连接云端服务器 " + user_ + "@" + host_ + "（端口 " + std::to_string(port_) + "）…");
  SetPhase(Phase::Connecting);
  StartSsh();
}

void Launcher::StartSsh() {
  ssh_ = std::make_unique<plat::Child>();
  tunnel_up_ = false;
  ssh_error_.clear();
  ssh_fatal_ = false;
  const std::string gl = std::to_string(local_gui_port_), cl = std::to_string(local_carsim_port_);
  // -F none: nothing from the user's own .ssh\config. The key and known_hosts by
  // relative names (cwd = their folder, see PrepareKey): UserKnownHostsFile splits at spaces.
  std::vector<std::string> argv = {ssh_exe_, "-F", "none", "-i", "remote_key", "-p", std::to_string(port_),
                                   "-o", "UserKnownHostsFile=known_hosts", "-o", "StrictHostKeyChecking=yes",
                                   "-o", "BatchMode=yes", "-o", "IdentitiesOnly=yes",
                                   "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
                                   "-o", "ExitOnForwardFailure=yes", "-o", "ConnectTimeout=20",
                                   "-o", "LogLevel=ERROR", "-T",
                                   "-L", gl + ":127.0.0.1:57120", "-L", cl + ":127.0.0.1:57121",
                                   user_ + "@" + host_};
  std::string err;
  next_probe_t_ = Clock::now() + std::chrono::seconds(2);
  if (!ssh_->Start(argv, ssh_cwd_, true, err)) {
    ssh_.reset();
    Fail("没能启动 SSH", err);
  }
}

void Launcher::RunService() {
  service_started_ = true;
  service_proc_ = std::make_unique<plat::Child>();
  std::vector<std::string> argv = py_argv_;
  for (const std::string& a : {std::string("-u"), std::string("carsim_service.py"), std::string("--port"),
                               std::to_string(local_carsim_port_)})
    argv.push_back(a);
  std::string err;
  if (!service_proc_->Start(argv, U8(fs::u8path(dir_) / "service"), true, err)) {
    service_proc_.reset();
    svc_ = {St::Fail, "没能启动：" + err};
    Log(kService, kErr, "没能启动 CarSim 服务：" + err);
    return;
  }
  svc_ = {St::Busy, "正在连接云端…"};
  Log(kService, kInfo, "CarSim 服务已启动（" + (py_argv_.size() > 1 ? "py -3" : py_exe_) + "）");
}

void Launcher::StartGui() {
  gui_proc_ = std::make_unique<plat::Child>();
  std::vector<std::string> argv = {gui_exe_, "--auto-connect"};
  argv.insert(argv.end(), gui_extra_.begin(), gui_extra_.end());
  std::string err;
  // Its output into the log: normally only errors (e.g. no OpenGL 3 on an old graphics driver).
  if (!gui_proc_->Start(argv, dir_, true, err)) {
    gui_proc_.reset();
    gui_ = {St::Fail, "没能打开：" + err};
    Log(kGui, kErr, "没能打开仿真界面：" + err);
    return;
  }
  gui_ = {St::Busy, "正在打开…"};
  Log(kGui, kInfo, "打开仿真界面");
}

void Launcher::Stop(const std::string& why, bool gui_closed) {
  if (!Running() || phase_ == Phase::Stopping) return;
  stop_note_ = why;
  stopping_gui_closed_ = gui_closed;
  Log(kLauncher, kInfo, why + "：断开云端连接…");
  if (service_proc_) service_proc_->Kill();
  if (gui_proc_ && gui_proc_->Running()) gui_proc_->RequestClose();
  if (ssh_) ssh_->CloseStdin();  // the server's session ends in order and stops the backend
  reconnect_pending_ = false;
  stop_deadline_ = Clock::now() + std::chrono::seconds(4);
  SetPhase(Phase::Stopping);
}

void Launcher::Fail(const std::string& title, const std::string& hint) {
  fail_title_ = title;
  fail_hint_ = hint;
  Log(kLauncher, kErr, title + (hint.empty() ? "" : "：" + hint));
  if (service_proc_) service_proc_->Kill();
  if (ssh_) {
    ssh_->CloseStdin();
    ssh_->Kill();
  }
  if (cloud_.st == St::Busy) cloud_.st = St::Fail;
  if (svc_.st == St::Busy || svc_.st == St::Ok) svc_ = {St::Idle, "已停止"};
  SetPhase(Phase::Failed);
  window_request_ = 2;
}

// ============================================================================
// per frame
// ============================================================================
void Launcher::OnCloudLine(const std::string& line) {
  const bool error = StartsWith(line, "[错误]");  // the server's own: e.g. its CARLA port is taken
  bool fatal = false;
  const std::string hint = SshHint(line, &fatal);
  // The first lasting reason stays (ssh follows a bad key file with "Permission denied").
  if ((error || !hint.empty()) && !ssh_fatal_) {
    ssh_error_ = error ? line.substr(std::string("[错误]").size()) : hint;
    ssh_fatal_ = error || fatal;
  }
  Log(kCloud, error || !hint.empty() ? kErr : kInfo, line);
  if (StartsWith(line, "已连上云端服务器")) {
    tunnel_up_ = true;
    const size_t a = line.find("（"), b = line.rfind("）");
    if (a != std::string::npos && b != std::string::npos && b > a) server_name_ = line.substr(a + 3, b - a - 3);
  }
  if (line == "就绪") {
    if (reconnects_ > 0) Log(kLauncher, kOk, "已重新连上云端（重连 " + std::to_string(reconnects_) + " 次）");
    server_ready_ = ready_once_ = true;
    reconnects_ = 0;
  } else if (Contains(line, "正在启动 CARLA")) {
    cloud_.detail = "正在启动 CARLA（首次约 1 分钟）";
  } else if (Contains(line, "已等待")) {
    // keeps the detail, the banner shows the time
  } else if (line == "CARLA 已就绪") {
    cloud_.detail = "CARLA 已就绪，正在启动后端";
  } else if (line == "后端已就绪") {
    cloud_.detail = "就绪";
  } else if (Contains(line, "后端正在重启")) {
    cloud_ = {St::Busy, "后端正在重启"};
  } else if (StartsWith(line, "后端已退出")) {
    cloud_ = {St::Warn, "后端退出了，正在重新启动"};
  } else if (Contains(line, "上一次的连接")) {
    cloud_.detail = "结束云端上一次的连接";
  }
}

void Launcher::OnServiceLine(const std::string& line) {
  int lvl = kInfo;
  if (StartsWith(line, "已连上云端")) {
    svc_ = {St::Ok, "已连上云端，等待运行"};
    lvl = kOk;
  } else if (StartsWith(line, "运行开始")) {
    svc_ = {St::Ok, "运行中"};
  } else if (line == "运行结束") {
    svc_ = {St::Ok, "已连上云端，等待运行"};
  } else if (StartsWith(line, "正在连接云端")) {
    svc_ = {St::Busy, "正在连接云端…"};
  } else if (StartsWith(line, "连接断开")) {
    svc_ = {St::Busy, "连接断开，正在重连…"};
    lvl = kWarn;
  } else if (StartsWith(line, "另一个 CarSim 服务")) {
    svc_ = {St::Fail, "另一台电脑的 CarSim 服务连上了云端"};
    lvl = kErr;
  } else if (!StartsWith(line, "已退出")) {
    lvl = kWarn;  // CarSim's own errors (the .sim, the solver) come as plain lines
  }
  Log(kService, lvl, line);
}

void Launcher::OnGuiLine(const std::string& line) {
  if (StartsWith(line, "fonts: ")) return;  // which font files it found: nothing for the user
  const bool err = StartsWith(line, "[error]") || StartsWith(line, "GLFW") || Contains(line, "error:");
  const bool warn = StartsWith(line, "[warn]") || StartsWith(line, "warning");
  Log(kGui, err ? kErr : warn ? kWarn : kInfo, line);
}

void Launcher::Update() {
  ++frame_;
  // numpy install
  if (pip_) {
    for (auto& l : pip_->TakeLines()) Log(kLauncher, kInfo, "pip: " + l);
    if (!pip_->Running()) {
      for (auto& l : pip_->TakeLines()) Log(kLauncher, kInfo, "pip: " + l);
      const int rc = pip_->ExitCode();
      pip_.reset();
      Log(kLauncher, rc == 0 ? kOk : kErr, rc == 0 ? "numpy 安装完成" : "numpy 安装失败（pip 退出码 " + std::to_string(rc) + "），原因见上面 pip 的输出");
      StartChecks();
    }
  }
  if (phase_ == Phase::Checking) {
    UpdateChecks();
    return;
  }

  // ---- children's output ----
  if (ssh_) for (auto& l : ssh_->TakeLines()) OnCloudLine(l);
  if (service_proc_) for (auto& l : service_proc_->TakeLines()) OnServiceLine(l);
  if (gui_proc_) for (auto& l : gui_proc_->TakeLines()) OnGuiLine(l);

  if (phase_ == Phase::Stopping) {
    const bool ssh_alive = ssh_ && ssh_->Running();
    const bool gui_alive = gui_proc_ && gui_proc_->Running();
    if ((!ssh_alive && !gui_alive) || Clock::now() > stop_deadline_) {
      if (ssh_) {
        for (auto& l : ssh_->TakeLines()) OnCloudLine(l);
        ssh_->Kill();
      }
      if (gui_proc_) {
        gui_proc_->Kill();
        for (auto& l : gui_proc_->TakeLines()) OnGuiLine(l);
      }
      ssh_.reset();
      service_proc_.reset();
      gui_proc_.reset();
      cloud_ = {St::Idle, "已断开"};
      svc_ = {St::Idle, "已停止"};
      gui_ = {St::Idle, stopping_gui_closed_ ? "已关闭" : "已关闭"};
      Log(kLauncher, kOk, "已断开云端连接（共运行 " + Elapsed(Since(start_t_)) + "）");
      SetPhase(Phase::Stopped);
      window_request_ = 2;
    }
    return;
  }
  // The GUI closed while nothing runs (e.g. after a failure): forget it, "重试" opens a new one.
  if (!Running() && gui_proc_ && !gui_proc_->Running()) {
    gui_proc_->Kill();
    for (auto& l : gui_proc_->TakeLines()) OnGuiLine(l);
    gui_proc_.reset();
    gui_ = {St::Idle, "已关闭"};
    Log(kGui, kInfo, "仿真界面已关闭");
  }
  if (!Running()) return;

  // ---- ssh ----
  if (reconnect_pending_) {
    if (Clock::now() >= reconnect_at_) {
      reconnect_pending_ = false;
      Log(kLauncher, kInfo, "重新连接云端（第 " + std::to_string(reconnects_) + " 次）…");
      StartSsh();
    }
  } else if (ssh_ && !ssh_->Running()) {
    for (auto& l : ssh_->TakeLines()) OnCloudLine(l);
    const int rc = ssh_->ExitCode();
    ssh_.reset();
    const bool had_tunnel = tunnel_up_;
    tunnel_up_ = server_ready_ = false;
    Log(kCloud, kWarn, "与云端的连接断开了（ssh 退出码 " + std::to_string(rc) + "）");
    // Connected before (a network hiccup, the server's session replaced...): again,
    // a few times. Never connected: the reason is permanent until fixed (key, network).
    if ((had_tunnel || ready_once_) && !ssh_fatal_ && reconnects_ < kMaxReconnects) {
      static const int kDelay[kMaxReconnects] = {3, 5, 10, 20, 30};  // s: about a minute in all
      reconnect_pending_ = true;
      reconnect_at_ = Clock::now() + std::chrono::seconds(kDelay[reconnects_]);
      ++reconnects_;
      cloud_ = {St::Warn, "连接断了，正在重连（第 " + std::to_string(reconnects_) + " 次）"};
      if (phase_ == Phase::Ready) gui_.detail = "已打开（等待云端重新连上）";
      return;
    }
    Fail(had_tunnel ? "与云端的连接断开了" : "没能连上云端服务器",
         !ssh_error_.empty() ? ssh_error_
                             : "网络不稳定或云服务器没有响应。检查网络后点“重试”；运行日志里有云端的提示。");
    return;
  }

  // Tunnel up: the server's first line; a probe of the local port as a fallback.
  if (!tunnel_up_ && ssh_ && Clock::now() >= next_probe_t_) {
    next_probe_t_ = Clock::now() + std::chrono::seconds(2);
    std::string err;
    plat::Socket s = plat::TcpConnect("127.0.0.1", local_gui_port_, err, 150);
    if (s != plat::kInvalidSocket) {
      plat::CloseSocket(s);
      tunnel_up_ = true;
    }
  }
  if (phase_ == Phase::Connecting) {
    if (tunnel_up_) {
      Log(kLauncher, kOk, "已连上云端" + (server_name_.empty() ? "" : "（" + server_name_ + "）"));
      cloud_ = {St::Busy, cloud_.detail == "正在连接…" ? "正在启动云端程序" : cloud_.detail};
      SetPhase(Phase::Starting);
    } else if (Since(phase_t_) > kConnectTimeout) {
      Fail("没能连上云端服务器", ssh_error_.empty() ? "90 秒内没有连上：检查网络；云服务器可能已关机或已到期。" : ssh_error_);
      return;
    }
  }
  if (phase_ == Phase::Starting || phase_ == Phase::Ready) {
    // Once per start: after a crash it waits for the button (the reason, e.g. the .sim, stays until fixed).
    if (tunnel_up_ && !service_proc_ && !service_started_) RunService();
    if (server_ready_ && cloud_.st != St::Ok) cloud_ = {St::Ok, "就绪" + (server_name_.empty() ? "" : "（" + server_name_ + "）")};
  }
  if (phase_ == Phase::Starting) {
    if (server_ready_) {
      Log(kLauncher, kOk, "云端就绪（用时 " + Elapsed(Since(start_t_)) + "）");
      SetPhase(Phase::Ready);
      if (!gui_proc_) StartGui();
    } else if (Since(phase_t_) > kReadyTimeout && !reconnect_pending_) {
      Fail("云端一直没有就绪", "7 分钟内云端没有准备好（CARLA 或后端没有启动成功）。运行日志里有云端的提示；可以点“重试”，还不行请联系管理员。");
      return;
    }
  }

  // ---- CarSim service ----
  if (service_proc_ && !service_proc_->Running()) {
    for (auto& l : service_proc_->TakeLines()) OnServiceLine(l);
    const int rc = service_proc_->ExitCode();
    service_proc_.reset();
    if (svc_.st != St::Fail) svc_ = {St::Fail, "已退出（退出码 " + std::to_string(rc) + "）"};
    Log(kService, kErr, "CarSim 服务退出了（退出码 " + std::to_string(rc) + "），原因见上面的输出");
  }

  // ---- GUI ----
  if (gui_proc_) {
    if (gui_proc_->Running()) {
      if (gui_.st == St::Busy && Since(phase_t_) > 2.0) gui_ = {St::Ok, "已打开"};  // (it opens in front)
    } else {
      const int rc = gui_proc_->ExitCode();
      gui_proc_->Kill();  // joins its output
      for (auto& l : gui_proc_->TakeLines()) OnGuiLine(l);
      gui_proc_.reset();
      gui_ = {rc == 0 ? St::Idle : St::Warn, rc == 0 ? "已关闭" : "异常退出（退出码 " + std::to_string(rc) + "）"};
      Log(kGui, rc == 0 ? kInfo : kWarn, rc == 0 ? "仿真界面已关闭" : "仿真界面异常退出（退出码 " + std::to_string(rc) + "）");
      Stop("仿真界面已关闭", true);
    }
  }
}

// ============================================================================
// UI
// ============================================================================
namespace {

ImVec4 StColor(Launcher::St s) {
  const ui::Palette& p = ui::Colors();
  switch (s) {
    case Launcher::St::Busy: return p.accent;
    case Launcher::St::Ok: return p.success;
    case Launcher::St::Warn: return p.warning;
    case Launcher::St::Fail: return p.danger;
    default: return p.text_dim;
  }
}

const char* StIcon(Launcher::St s) {
  switch (s) {
    case Launcher::St::Ok: return ICON_FA_CIRCLE_CHECK;
    case Launcher::St::Warn: return ICON_FA_TRIANGLE_EXCLAMATION;
    case Launcher::St::Fail: return ICON_FA_CIRCLE_XMARK;
    case Launcher::St::Busy: return ICON_FA_SPINNER;
    default: return ICON_FA_CIRCLE;
  }
}

ImVec4 Mix(const ImVec4& a, const ImVec4& b, float t) {
  return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

void Spinner(ImDrawList* dl, ImVec2 c, float r, float thick, ImU32 col) {
  const float t = static_cast<float>(ImGui::GetTime());
  const float a0 = t * 5.5f, a1 = a0 + 1.9f + 0.9f * std::sin(t * 2.2f);
  dl->PathClear();
  for (int i = 0; i <= 24; ++i) {
    const float a = a0 + (a1 - a0) * i / 24.0f;
    dl->PathLineTo(ImVec2(c.x + std::cos(a) * r, c.y + std::sin(a) * r));
  }
  dl->PathStroke(col, 0, thick);
}

// Text centred on x, clipped to width w.
void CenterText(ImDrawList* dl, ImFont* f, float size, float cx, float y, float w, ImU32 col, const std::string& s) {
  std::string t = s;
  ImVec2 ts = f->CalcTextSizeA(size, FLT_MAX, 0, t.c_str());
  while (ts.x > w && t.size() > 3) {
    // drop the last UTF-8 character
    size_t cut = t.size() - 1;
    while (cut > 0 && (static_cast<unsigned char>(t[cut]) & 0xC0) == 0x80) --cut;
    t = t.substr(0, cut);
    ts = f->CalcTextSizeA(size, FLT_MAX, 0, (t + "…").c_str());
    if (ts.x <= w) {
      t += "…";
      break;
    }
  }
  dl->AddText(f, size, ImVec2(cx - ts.x * 0.5f, y), col, t.c_str());
}

}  // namespace

// A section card with a title bar, rounded like the step bar and the banner.
static void CardBegin(const char* icon, const char* title, const char* id) {
  const ui::Palette& p = ui::Colors();
  const float s = ui::Scale(), fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.card);
  ImGui::PushStyleColor(ImGuiCol_Border, p.card_border);
  ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8 * s);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14 * s, 10 * s));
  ImGui::BeginChild(id, ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
  const ImVec2 wp = ImGui::GetWindowPos();
  const float w = ImGui::GetWindowWidth(), h = fs * 2.2f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(ImVec2(wp.x + 1, wp.y + 1), ImVec2(wp.x + w - 1, wp.y + h), ImGui::GetColorU32(p.header), 8 * s,
                    ImDrawFlags_RoundCornersTop);
  dl->AddLine(ImVec2(wp.x + 1, wp.y + h), ImVec2(wp.x + w - 1, wp.y + h), ImGui::GetColorU32(p.card_border));
  const float ty = wp.y + (h - fs) * 0.5f;
  dl->AddText(ImVec2(wp.x + 14 * s, ty), ImGui::GetColorU32(p.accent), icon);
  ImFont* bold = ui::GetFonts().bold ? ui::GetFonts().bold : ImGui::GetFont();
  dl->AddText(bold, fs, ImVec2(wp.x + 14 * s + fs * 1.6f, ty), ImGui::GetColorU32(p.text), title);
  ImGui::SetCursorPosY(h + 10 * s);
}

static void CardEnd() {
  ImGui::Dummy(ImVec2(0, 2 * ui::Scale()));
  ImGui::EndChild();
  ImGui::PopStyleVar(2);
  ImGui::PopStyleColor(2);
  ImGui::Dummy(ImVec2(0, 4 * ui::Scale()));
}

bool Launcher::Btn(const char* id, const char* icon, const char* text, int kind, float w) {
  ImGui::PushID(id);
  const float s = ui::Scale();
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12 * s, 6 * s));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5 * s);
  const bool r = ui::Button(icon, text, static_cast<ui::Kind>(kind), ImVec2(w, 0));
  ImGui::PopStyleVar(2);
  ui::RecordTarget(id);
  ImGui::PopID();
  return r;
}

void Launcher::Frame() {
  Update();
  TourTick();
  const ui::Palette& p = ui::Colors();
  const float s = ui::Scale();
  ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(vp->WorkSize);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin("##launcher", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleVar();
  DrawHeader();
  const float pad = 18 * s;
  ImGui::SetCursorPos(ImVec2(pad, ImGui::GetCursorPosY() + 12 * s));
  ImGui::BeginChild("##body", ImVec2(vp->WorkSize.x - 2 * pad, vp->WorkSize.y - ImGui::GetCursorPosY() - 14 * s),
                    ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
  DrawSteps();
  ImGui::Dummy(ImVec2(0, 10 * s));
  DrawBanner();
  ImGui::Dummy(ImVec2(0, 10 * s));
  const float w = ImGui::GetContentRegionAvail().x;
  const bool wide = w > 820 * s;
  if (wide) {
    const float left = std::floor(w * 0.56f);
    ImGui::BeginGroup();
    ImGui::PushItemWidth(left);
    ImGui::BeginChild("##checks", ImVec2(left, 0), ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoBackground);
    DrawChecks();
    ImGui::EndChild();
    ImGui::PopItemWidth();
    ImGui::EndGroup();
    ImGui::SameLine(0, 12 * s);
    ImGui::BeginChild("##settings", ImVec2(0, 0), ImGuiChildFlags_AutoResizeY, ImGuiWindowFlags_NoBackground);
    DrawSettings();
    ImGui::EndChild();
  } else {
    DrawChecks();
    DrawSettings();
  }
  DrawLog();
  ImGui::EndChild();
  DrawModals();
  ImGui::End();
  (void)p;
}

void Launcher::DrawHeader() {
  const ui::Palette& p = ui::Colors();
  const ui::Fonts& f = ui::GetFonts();
  const float s = ui::Scale();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 wp = ImGui::GetWindowPos();
  const float W = ImGui::GetWindowWidth(), H = 64 * s;
  dl->AddRectFilled(wp, ImVec2(wp.x + W, wp.y + H), ImGui::GetColorU32(p.chrome));
  dl->AddLine(ImVec2(wp.x, wp.y + H), ImVec2(wp.x + W, wp.y + H), ImGui::GetColorU32(p.card_border));
  // Logo: a rounded tile in the accent colour with a car.
  const float L = 40 * s;
  const ImVec2 lo(wp.x + 18 * s, wp.y + (H - L) * 0.5f);
  dl->AddRectFilled(lo, ImVec2(lo.x + L, lo.y + L), ImGui::GetColorU32(p.accent), 10 * s);
  dl->AddRectFilled(lo, ImVec2(lo.x + L, lo.y + L * 0.5f), IM_COL32(255, 255, 255, 28), 10 * s, ImDrawFlags_RoundCornersTop);
  ImFont* tf = f.title ? f.title : ImGui::GetFont();
  const float ts = tf->FontSize;
  const ImVec2 ic = tf->CalcTextSizeA(ts, FLT_MAX, 0, ICON_FA_CAR_SIDE);
  dl->AddText(tf, ts, ImVec2(lo.x + (L - ic.x) * 0.5f, lo.y + (L - ts) * 0.5f), IM_COL32_WHITE, ICON_FA_CAR_SIDE);
  const float tx = lo.x + L + 14 * s;
  dl->AddText(tf, ts, ImVec2(tx, wp.y + 11 * s), ImGui::GetColorU32(p.text), "CARLA CoSim Studio");
  const float tw = tf->CalcTextSizeA(ts, FLT_MAX, 0, "CARLA CoSim Studio").x;
  ImFont* bf = f.bold ? f.bold : ImGui::GetFont();
  const float fs = ImGui::GetFontSize();
  const char* tag = "远程启动器";
  const ImVec2 tg = bf->CalcTextSizeA(fs * 0.86f, FLT_MAX, 0, tag);
  const ImVec2 ta(tx + tw + 10 * s, wp.y + 13 * s);
  dl->AddRectFilled(ta, ImVec2(ta.x + tg.x + 14 * s, ta.y + tg.y + 6 * s), ImGui::GetColorU32(ui::WithAlpha(p.accent, 0.16f)), 10 * s);
  dl->AddText(bf, fs * 0.86f, ImVec2(ta.x + 7 * s, ta.y + 3 * s), ImGui::GetColorU32(p.accent), tag);
  dl->AddText(ImGui::GetFont(), fs * 0.92f, ImVec2(tx, wp.y + 11 * s + ts + 4 * s), ImGui::GetColorU32(p.text_dim),
              "这台电脑运行 CarSim 和仿真界面 · CARLA、后端和控制算法在云服务器上");
  // Right: help, logs, theme.
  const float bh = ImGui::GetFrameHeight();
  float x = W - 18 * s;
  auto right_button = [&](const char* id, const char* icon, const char* label, const char* tip) {
    const std::string text = std::string(icon) + "  " + label;
    const float bw = ImGui::CalcTextSize(text.c_str()).x + 20 * s;
    x -= bw;
    ImGui::SetCursorPos(ImVec2(x, (H - bh) * 0.5f));
    x -= 6 * s;
    ImGui::PushID(id);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
    const bool r = ImGui::Button(text.c_str(), ImVec2(bw, 0));
    ImGui::PopStyleColor(2);
    ui::RecordTarget(id);
    if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
    ImGui::PopID();
    return r;
  };
  if (right_button("hdr:theme", dark_ ? ICON_FA_SUN : ICON_FA_MOON, dark_ ? "浅色" : "深色", "切换浅色 / 深色界面")) {
    dark_ = !dark_;
    theme_changed_ = true;
    SaveSettings();
  }
  if (right_button("hdr:logs", ICON_FA_FOLDER_OPEN, "日志", ("打开日志文件夹：" + state_dir_).c_str()))
    plat::OpenWithSystem(state_dir_);
  if (right_button("hdr:help", ICON_FA_BOOK_OPEN, "使用说明", "打开使用说明（常见问题也在里面）")) {
    if (!plat::OpenWithSystem(plat::FileExists(notes_) ? notes_ : kDocUrl)) plat::OpenWithSystem(kDocUrl);
  }
  ImGui::SetCursorPos(ImVec2(0, H));
  ImGui::Dummy(ImVec2(0, 0));
}

void Launcher::DrawSteps() {
  const ui::Palette& p = ui::Colors();
  const ui::Fonts& f = ui::GetFonts();
  const float s = ui::Scale();
  const float fs = ImGui::GetFontSize();
  St env = St::Busy;
  if (phase_ != Phase::Checking) {
    env = St::Ok;
    for (const auto& c : checks_)
      if (c.st == St::Fail) env = St::Fail;
    if (pip_) env = St::Busy;
  }
  std::string env_detail = env == St::Ok ? "全部通过" : env == St::Fail ? "有问题要处理" : "正在检查…";
  if (env == St::Fail) {
    int n = 0;
    for (const auto& c : checks_) n += c.st == St::Fail;
    env_detail = std::to_string(n) + " 项需要处理";
  }
  struct Node {
    const char* icon;
    const char* name;
    St st;
    std::string detail;
  } nodes[4] = {{ICON_FA_LIST_CHECK, "环境检查", env, env_detail},
                {ICON_FA_CLOUD, "云端连接", cloud_.st, cloud_.detail.empty() ? "未连接" : cloud_.detail},
                {ICON_FA_GEARS, "CarSim 服务", svc_.st, svc_.detail.empty() ? "未启动" : svc_.detail},
                {ICON_FA_DISPLAY, "仿真界面", gui_.st, gui_.detail.empty() ? "未打开" : gui_.detail}};
  if (!Running() && phase_ != Phase::Stopped && phase_ != Phase::Failed) {
    nodes[1].detail = "未连接";
    nodes[2].detail = "未启动";
    nodes[3].detail = "未打开";
  }
  const float W = ImGui::GetContentRegionAvail().x;
  const float H = 92 * s;
  const ImVec2 o = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(o, ImVec2(o.x + W, o.y + H), ImGui::GetColorU32(p.card), 8 * s);
  dl->AddRect(o, ImVec2(o.x + W, o.y + H), ImGui::GetColorU32(p.card_border), 8 * s);
  const float col = W / 4.0f, R = 17 * s, cy = o.y + 12 * s + R;
  for (int i = 0; i < 4; ++i) {
    const float cx = o.x + col * (i + 0.5f);
    if (i < 3) {  // connector to the next node
      const float x0 = cx + R + 8 * s, x1 = cx + col - R - 8 * s;
      const bool done = nodes[i].st == St::Ok && nodes[i + 1].st != St::Idle;
      dl->AddLine(ImVec2(x0, cy), ImVec2(x1, cy), ImGui::GetColorU32(done ? ui::WithAlpha(p.success, 0.8f) : p.card_border), 2 * s);
    }
    const ImVec4 c = StColor(nodes[i].st);
    const bool active = nodes[i].st != St::Idle;
    dl->AddCircleFilled(ImVec2(cx, cy), R, ImGui::GetColorU32(active ? ui::WithAlpha(c, 0.16f) : p.field), 40);
    dl->AddCircle(ImVec2(cx, cy), R, ImGui::GetColorU32(active ? c : p.card_border), 40, 1.5f * s);
    if (nodes[i].st == St::Busy) Spinner(dl, ImVec2(cx, cy), R + 4 * s, 2.5f * s, ImGui::GetColorU32(p.accent));
    const char* icon = nodes[i].st == St::Ok ? ICON_FA_CHECK : nodes[i].st == St::Fail ? ICON_FA_XMARK : nodes[i].icon;
    const ImVec2 is = ImGui::CalcTextSize(icon);
    dl->AddText(ImVec2(cx - is.x * 0.5f, cy - is.y * 0.5f), ImGui::GetColorU32(active ? c : p.text_dim), icon);
    ImFont* bf = f.bold ? f.bold : ImGui::GetFont();
    CenterText(dl, bf, fs, cx, cy + R + 7 * s, col - 12 * s, ImGui::GetColorU32(p.text), nodes[i].name);
    CenterText(dl, ImGui::GetFont(), fs * 0.86f, cx, cy + R + 9 * s + fs, col - 12 * s,
               ImGui::GetColorU32(nodes[i].st == St::Idle ? p.text_dim : Mix(c, p.text, 0.25f)), nodes[i].detail);
  }
  ImGui::Dummy(ImVec2(W, H));
}

void Launcher::DrawBanner() {
  const ui::Palette& p = ui::Colors();
  const ui::Fonts& f = ui::GetFonts();
  const float s = ui::Scale();
  const float fs = ImGui::GetFontSize();
  St st = St::Idle;
  std::string title, detail;
  const std::string server = user_ + "@" + host_ + ":" + std::to_string(port_);
  switch (phase_) {
    case Phase::Checking:
      st = St::Busy;
      title = "正在检查这台电脑的环境…";
      detail = "启动包文件、SSH、Python 和 numpy、本机端口";
      break;
    case Phase::Idle:
      if (checks_ok_) {
        st = St::Ok;
        title = "一切就绪，可以启动";
        detail = "点“启动”：连接云端服务器 " + server + "，然后自动打开仿真界面";
      } else {
        st = St::Fail;
        title = "还不能启动：请先处理下面标红的项目";
        detail = "处理好以后点“重新检查”";
      }
      break;
    case Phase::Connecting:
      st = St::Busy;
      title = "正在连接云端服务器…";
      detail = server + " · 已用 " + Elapsed(Since(start_t_));
      break;
    case Phase::Starting:
      st = reconnect_pending_ ? St::Warn : St::Busy;
      title = reconnect_pending_ ? "与云端的连接断了，正在重连…" : "云端正在准备：" + cloud_.detail;
      detail = "已用 " + Elapsed(Since(start_t_)) + " · 首次启动 CARLA 约需 1 分钟，就绪后自动打开仿真界面";
      break;
    case Phase::Ready:
      if (reconnect_pending_ || cloud_.st == St::Warn || !server_ready_) {
        st = St::Warn;
        title = "与云端的连接断了，正在重连…";
        detail = "仿真界面会在云端重新连上后自动恢复连接";
      } else if (svc_.st == St::Fail) {
        st = St::Warn;
        title = "CarSim 服务停止了";
        detail = svc_.detail + " · 运行日志里有原因；点“重启 CarSim 服务”再试";
      } else {
        st = St::Ok;
        title = gui_.st == St::Busy ? "就绪：正在打开仿真界面…" : "就绪：可以在仿真界面里开始仿真了";
        detail = "已运行 " + Elapsed(Since(start_t_)) + " · 关闭仿真界面后会自动断开云端连接";
      }
      break;
    case Phase::Stopping:
      st = St::Busy;
      title = "正在断开…";
      detail = "停止 CarSim 服务，结束云端的会话";
      break;
    case Phase::Stopped:
      st = St::Idle;
      title = stop_note_.empty() ? "已断开" : stop_note_ + "，云端连接已断开";
      detail = "需要继续仿真时点“再次启动”";
      break;
    case Phase::Failed:
      st = St::Fail;
      title = fail_title_;
      detail = fail_hint_;
      break;
  }
  const ImVec4 c = StColor(st);
  const float W = ImGui::GetContentRegionAvail().x;
  const ImVec2 o = ImGui::GetCursorScreenPos();
  // buttons first (their width limits the text)
  struct B {
    const char* id;
    const char* icon;
    const char* text;
    int kind;
  };
  std::vector<B> bs;
  if (phase_ == Phase::Idle && checks_ok_) bs.push_back({"banner:start", ICON_FA_PLAY, "启动", 0});
  if (phase_ == Phase::Idle && !checks_ok_) bs.push_back({"banner:recheck", ICON_FA_ROTATE_RIGHT, "重新检查", 1});
  if (phase_ == Phase::Starting && svc_.st == St::Fail && !service_proc_)
    bs.push_back({"banner:restart_service", ICON_FA_ROTATE_RIGHT, "重启 CarSim 服务", 0});
  if (phase_ == Phase::Connecting || phase_ == Phase::Starting) bs.push_back({"banner:stop", ICON_FA_STOP, "取消", 1});
  if (phase_ == Phase::Ready) {
    if (svc_.st == St::Fail && !service_proc_) bs.push_back({"banner:restart_service", ICON_FA_ROTATE_RIGHT, "重启 CarSim 服务", 0});
    if (!gui_proc_) bs.push_back({"banner:open_gui", ICON_FA_DISPLAY, "打开仿真界面", 0});
    bs.push_back({"banner:stop", ICON_FA_STOP, "停止并断开", 2});
  }
  if (phase_ == Phase::Stopped) bs.push_back({"banner:start", ICON_FA_PLAY, "再次启动", 0});
  if (phase_ == Phase::Failed) {
    bs.push_back({"banner:retry", ICON_FA_ROTATE_RIGHT, "重试", 0});
  }
  float bw_total = 0;
  const float bpad = 24 * s;
  for (const auto& b : bs) bw_total += ImGui::CalcTextSize((std::string(b.icon) + "  " + b.text).c_str()).x + bpad + 8 * s;
  const float text_w = W - 90 * s - bw_total - 16 * s;
  ImFont* tf = f.title ? f.title : ImGui::GetFont();
  const float tsz = tf->FontSize;
  const ImVec2 dsz = ImGui::GetFont()->CalcTextSizeA(fs * 0.95f, FLT_MAX, text_w, detail.c_str());
  const float H = std::max(84 * s, 22 * s + tsz + 6 * s + dsz.y + 18 * s);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec4 bg = Mix(p.card, c, ui::IsDark() ? 0.10f : 0.07f);
  dl->AddRectFilled(o, ImVec2(o.x + W, o.y + H), ImGui::GetColorU32(bg), 8 * s);
  dl->AddRect(o, ImVec2(o.x + W, o.y + H), ImGui::GetColorU32(ui::WithAlpha(c, 0.45f)), 8 * s);
  dl->AddRectFilled(o, ImVec2(o.x + 5 * s, o.y + H), ImGui::GetColorU32(c), 8 * s, ImDrawFlags_RoundCornersLeft);
  // icon
  const ImVec2 ic(o.x + 46 * s, o.y + H * 0.5f);
  if (st == St::Busy) {
    Spinner(dl, ic, 17 * s, 3.5f * s, ImGui::GetColorU32(c));
  } else {
    const char* icon = st == St::Ok ? ICON_FA_CIRCLE_CHECK : st == St::Fail ? ICON_FA_CIRCLE_XMARK
                       : st == St::Warn ? ICON_FA_TRIANGLE_EXCLAMATION : ICON_FA_CIRCLE_PAUSE;
    const float isz = tsz * 1.45f;
    const ImVec2 is = tf->CalcTextSizeA(isz, FLT_MAX, 0, icon);
    dl->AddText(tf, isz, ImVec2(ic.x - is.x * 0.5f, ic.y - is.y * 0.5f), ImGui::GetColorU32(c), icon);
  }
  const float tx = o.x + 80 * s;
  const float ty = o.y + (H - (tsz + 6 * s + dsz.y)) * 0.5f;
  dl->AddText(tf, tsz, ImVec2(tx, ty), ImGui::GetColorU32(p.text), title.c_str());
  dl->AddText(ImGui::GetFont(), fs * 0.95f, ImVec2(tx, ty + tsz + 6 * s), ImGui::GetColorU32(p.text_dim), detail.c_str(),
              nullptr, text_w);
  // buttons, right-aligned, vertically centred
  const ImVec2 wpos = ImGui::GetWindowPos();
  float x = o.x + W - 18 * s;
  for (auto it = bs.rbegin(); it != bs.rend(); ++it) x -= ImGui::CalcTextSize((std::string(it->icon) + "  " + it->text).c_str()).x + bpad + 8 * s;
  const float bh = ImGui::GetFontSize() + 12 * s;
  for (const auto& b : bs) {
    ImGui::SetCursorScreenPos(ImVec2(x + 8 * s, o.y + (H - bh) * 0.5f));
    const float bw = ImGui::CalcTextSize((std::string(b.icon) + "  " + b.text).c_str()).x + bpad;
    if (Btn(b.id, b.icon, b.text, b.kind, bw)) {
      const std::string id = b.id;
      if (id == "banner:start" || id == "banner:retry") {
        if (id == "banner:retry" && !checks_ok_) StartChecks();
        else Start();
      } else if (id == "banner:recheck") {
        StartChecks();
      } else if (id == "banner:stop") {
        if (gui_proc_ && gui_proc_->Running()) ask_stop_ = true;
        else Stop(phase_ == Phase::Ready ? "已停止" : "已取消", false);
      } else if (id == "banner:restart_service") {
        RunService();
      } else if (id == "banner:open_gui") {
        StartGui();
      }
    }
    x += bw + 8 * s;
  }
  (void)wpos;
  ImGui::SetCursorScreenPos(o);
  ImGui::Dummy(ImVec2(W, H));
}

void Launcher::DrawChecks() {
  const ui::Palette& p = ui::Colors();
  const float s = ui::Scale();
  const float fs = ImGui::GetFontSize();
  CardBegin(ICON_FA_LIST_CHECK, "这台电脑的环境", "card:checks");
  const bool can_act = !Running() && phase_ != Phase::Checking && !pip_;
  for (size_t i = 0; i < checks_.size(); ++i) {
    const Check& c = checks_[i];
    ImGui::PushID(static_cast<int>(i));
    const float x0 = ImGui::GetCursorPosX();
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec4 col = StColor(c.st);
    if (c.st == St::Busy) {
      Spinner(ImGui::GetWindowDrawList(), ImVec2(pos.x + fs * 0.5f, pos.y + fs * 0.62f), fs * 0.42f, 2.0f * s,
              ImGui::GetColorU32(col));
      ImGui::Dummy(ImVec2(fs, fs));
    } else {
      ImGui::TextColored(col, "%s", StIcon(c.st));
    }
    ImGui::SameLine(x0 + fs * 1.6f);
    if (ui::GetFonts().bold) ImGui::PushFont(ui::GetFonts().bold);
    ImGui::TextUnformatted(c.name.c_str());
    if (ui::GetFonts().bold) ImGui::PopFont();
    ImGui::SameLine(x0 + fs * 7.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, c.st == St::Fail ? p.danger : p.text_dim);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(c.detail.c_str());
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    if (c.st == St::Fail && !c.hint.empty()) {
      ImGui::Indent(fs * 1.6f);
      ImGui::PushStyleColor(ImGuiCol_Text, Mix(p.text, p.warning, 0.55f));
      ImGui::PushTextWrapPos(0.0f);
      ImGui::TextUnformatted(c.hint.c_str());
      ImGui::PopTextWrapPos();
      ImGui::PopStyleColor();
      if (!can_act) ImGui::BeginDisabled();
      if (i == kSsh) {
#ifdef _WIN32
        if (Btn("check:ssh_settings", ICON_FA_UP_RIGHT_FROM_SQUARE, "打开“可选功能”设置", 1))
          plat::OpenWithSystem("ms-settings:optionalfeatures");
#endif
      } else if (i == kPython) {
        if (Btn("check:py_download", ICON_FA_DOWNLOAD, "下载 Python", 1)) plat::OpenWithSystem(kPythonUrl);
        ImGui::SameLine();
        if (Btn("check:py_pick", ICON_FA_FOLDER_OPEN, "选择 python.exe…", 1)) {
          std::string path, err;
#ifdef _WIN32
          const char* ext = "exe";
#else
          const char* ext = "";
#endif
          if (plat::PickFile("选择 python.exe", ext, path, err)) {
            python_setting_ = path;
            SaveSettings();
            Log(kLauncher, kInfo, "使用指定的 Python：" + path);
            StartChecks();
          } else if (!err.empty()) {
            Log(kLauncher, kWarn, err);
          }
        }
      } else if (i == kNumpy) {
        if (Btn("check:numpy", ICON_FA_DOWNLOAD, "安装 numpy", 0)) InstallNumpy();
      } else if (i == kPorts && !busy_pids_.empty()) {
        if (Btn("check:kill_old", ICON_FA_PLUG_CIRCLE_XMARK, "结束旧连接", 0)) {
          for (long pid : busy_pids_) {
#ifdef _WIN32
            if (HANDLE h = OpenProcess(PROCESS_TERMINATE, FALSE, static_cast<DWORD>(pid))) {
              TerminateProcess(h, 1);
              CloseHandle(h);
            }
#else
            kill(static_cast<pid_t>(pid), SIGTERM);
#endif
            Log(kLauncher, kInfo, "结束了占用端口的旧连接（ssh，PID " + std::to_string(pid) + "）");
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          StartChecks();
        }
      }
      if (!can_act) ImGui::EndDisabled();
      ImGui::Unindent(fs * 1.6f);
    }
    ImGui::Dummy(ImVec2(0, 2 * s));
    ImGui::PopID();
  }
  ImGui::Dummy(ImVec2(0, 2 * s));
  if (!can_act) ImGui::BeginDisabled();
  if (Btn("check:recheck", ICON_FA_ROTATE_RIGHT, "重新检查", 1)) StartChecks();
  if (!can_act) ImGui::EndDisabled();
  CardEnd();
}

void Launcher::DrawSettings() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  CardBegin(ICON_FA_SLIDERS, "连接与设置", "card:settings");
  const float x0 = ImGui::GetCursorPosX();
  auto row = [&](const char* label) {
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, p.text_dim);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    ImGui::SameLine(x0 + fs * 5.2f);
  };
  row("云服务器");
  ImGui::TextUnformatted((user_ + "@" + host_ + (port_ ? ":" + std::to_string(port_) : "")).c_str());
  row("");
  ui::DimWrapped("地址和钥匙在启动包里设定；服务器换了请用新的启动包");
  row("本机端口");
  ImGui::Text("%d（界面）· %d（CarSim 服务）", local_gui_port_, local_carsim_port_);
  ImGui::Dummy(ImVec2(0, 2));
  row("Python");
  const bool can = !Running() && phase_ != Phase::Checking && !pip_;
  if (!can) ImGui::BeginDisabled();
  if (python_setting_.empty()) {
    ImGui::TextUnformatted("自动查找（py -3，再 python）");
  } else {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(python_setting_.c_str());
    ImGui::PopTextWrapPos();
  }
  row("");
  if (Btn("set:py_pick", ICON_FA_FOLDER_OPEN, "选择…", 1)) {
    std::string path, err;
#ifdef _WIN32
    const char* ext = "exe";
#else
    const char* ext = "";
#endif
    if (plat::PickFile("选择 python.exe（装有 numpy 的 64 位 Python）", ext, path, err)) {
      python_setting_ = path;
      SaveSettings();
      Log(kLauncher, kInfo, "使用指定的 Python：" + path);
      StartChecks();
    } else if (!err.empty()) {
      Log(kLauncher, kWarn, err);
    }
  }
  if (!python_setting_.empty()) {
    ImGui::SameLine();
    if (Btn("set:py_auto", ICON_FA_WAND_MAGIC_SPARKLES, "改回自动", 1)) {
      python_setting_.clear();
      SaveSettings();
      Log(kLauncher, kInfo, "Python 改回自动查找");
      StartChecks();
    }
  }
  if (!can) ImGui::EndDisabled();
  ImGui::Dummy(ImVec2(0, 2));
  row("运行记录");
  ui::DimWrapped("保存在云服务器上（carsim_carla_bridge/runs/…），可在仿真界面的“数据浏览”里查看");
  CardEnd();
}

void Launcher::DrawLog() {
  const ui::Palette& p = ui::Colors();
  const float s = ui::Scale();
  const float fs = ImGui::GetFontSize();
  ImGui::Dummy(ImVec2(0, 2 * s));
  const float W = ImGui::GetContentRegionAvail().x;
  const float H = std::max(ImGui::GetContentRegionAvail().y, 150 * s);
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.card);
  ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8 * s);
  ImGui::BeginChild("##logcard", ImVec2(W, H), ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar);
  ImGui::SetCursorPos(ImVec2(12 * s, 8 * s));
  ImGui::TextColored(p.accent, ICON_FA_TERMINAL);
  ImGui::SameLine();
  if (ui::GetFonts().bold) ImGui::PushFont(ui::GetFonts().bold);
  ImGui::TextUnformatted("运行日志");
  if (ui::GetFonts().bold) ImGui::PopFont();
  ImGui::SameLine(0, 16 * s);
  static const char* kTabs[] = {"全部", "云端", "CarSim 服务", "仿真界面", "启动器"};
  static const int kTabSrc[] = {-1, kCloud, kService, kGui, kLauncher};
  for (int i = 0; i < 5; ++i) {
    const bool on = log_filter_ == kTabSrc[i];
    ImGui::PushStyleColor(ImGuiCol_Button, on ? ui::WithAlpha(p.accent, 0.22f) : ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Border, on ? ui::WithAlpha(p.accent, 0.6f) : ImVec4(0, 0, 0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 10 * s);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(9 * s, 2 * s));
    ImGui::SameLine(0, 4 * s);
    const std::string id = std::string(kTabs[i]) + "##logtab";
    if (ImGui::Button(id.c_str())) {
      log_filter_ = kTabSrc[i];
      log_follow_ = true;
    }
    ui::RecordTarget(std::string("log:tab") + std::to_string(i));
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
  }
  const char* copy = ICON_FA_COPY "  复制";
  const float cw = ImGui::CalcTextSize(copy).x + 16 * s;
  ImGui::SameLine(W - cw - 12 * s);
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8 * s, 2 * s));
  if (ImGui::Button(copy)) {
    std::string all;
    static const char* kSrc[] = {"启动器", "云端", "CarSim", "界面"};
    for (const auto& l : log_)
      if (log_filter_ < 0 || l.src == log_filter_) all += l.time + " " + kSrc[l.src] + "  " + l.text + "\n";
    ImGui::SetClipboardText(all.c_str());
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("把显示的日志复制到剪贴板（发给别人帮忙看问题时用）");
  ImGui::PopStyleVar();
  ImGui::SetCursorPosX(0);
  ImGui::Dummy(ImVec2(0, 2 * s));
  const ImVec2 lp = ImGui::GetCursorScreenPos();
  ImGui::GetWindowDrawList()->AddLine(ImVec2(lp.x, lp.y), ImVec2(lp.x + W, lp.y), ImGui::GetColorU32(p.card_border));
  ImGui::SetCursorPos(ImVec2(12 * s, ImGui::GetCursorPosY() + 4 * s));
  ImGui::BeginChild("##loglines", ImVec2(W - 16 * s, ImGui::GetContentRegionAvail().y - 6 * s), ImGuiChildFlags_None);
  ImGui::Dummy(ImVec2(0, 2 * s));
  static const char* kSrcName[] = {"启动器", "云端", "CarSim", "界面"};
  const ImVec4 src_col[] = {p.text_dim, Mix(p.accent, p.text, 0.2f), Mix(ImVec4(0.66f, 0.49f, 0.90f, 1), p.text, 0.2f),
                            Mix(ImVec4(0.93f, 0.55f, 0.24f, 1), p.text, 0.2f)};
  ImFont* mono = ui::GetFonts().mono;
  for (const auto& l : log_) {
    if (log_filter_ >= 0 && l.src != log_filter_) continue;
    if (mono) ImGui::PushFont(mono);
    ImGui::TextColored(ui::WithAlpha(p.text_dim, 0.8f), "%s", l.time.c_str());
    if (mono) ImGui::PopFont();
    ImGui::SameLine(fs * 4.6f);
    ImGui::TextColored(src_col[l.src], "%s", kSrcName[l.src]);
    ImGui::SameLine(fs * 8.4f);
    const ImVec4 tc = l.lvl == kErr ? p.danger : l.lvl == kWarn ? p.warning : l.lvl == kOk ? p.success : p.text;
    ImGui::PushStyleColor(ImGuiCol_Text, tc);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(l.text.c_str());
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
  }
  // Follow new lines unless the user scrolled up to read.
  if (ImGui::GetScrollY() < ImGui::GetScrollMaxY() - 4) {
    if (ImGui::IsWindowHovered() && ImGui::GetIO().MouseWheel != 0) log_follow_ = false;
  } else {
    log_follow_ = true;
  }
  if (log_follow_ && log_.size() != log_seen_) ImGui::SetScrollHereY(1.0f);
  log_seen_ = log_.size();
  ImGui::EndChild();
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}

void Launcher::DrawModals() {
  const ui::Palette& p = ui::Colors();
  const float s = ui::Scale();
  auto modal = [&](const char* id, bool& flag, const char* title, const std::string& text, const char* ok_id,
                   const char* ok_text, const std::function<void()>& on_ok) {
    if (flag && !ImGui::IsPopupOpen(id)) ImGui::OpenPopup(id);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(460 * s, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20 * s, 16 * s));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8 * s);
    if (ImGui::BeginPopupModal(id, nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize)) {
      if (ui::GetFonts().title) ImGui::PushFont(ui::GetFonts().title);
      ImGui::TextColored(p.warning, ICON_FA_TRIANGLE_EXCLAMATION);
      ImGui::SameLine();
      ImGui::TextUnformatted(title);
      if (ui::GetFonts().title) ImGui::PopFont();
      ImGui::Dummy(ImVec2(0, 4 * s));
      ImGui::PushTextWrapPos(0.0f);
      ImGui::TextUnformatted(text.c_str());
      ImGui::PopTextWrapPos();
      ImGui::Dummy(ImVec2(0, 10 * s));
      const float bw = 130 * s;
      ImGui::SetCursorPosX(ImGui::GetWindowWidth() - 2 * bw - 28 * s);
      if (Btn(ok_id, ICON_FA_POWER_OFF, ok_text, 2, bw)) {
        flag = false;
        ImGui::CloseCurrentPopup();
        on_ok();
      }
      ImGui::SameLine();
      if (Btn("modal:cancel", "", "取消", 1, bw) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        flag = false;
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
    ImGui::PopStyleVar(2);
  };
  const bool gui_open = gui_proc_ && gui_proc_->Running();
  modal("##quit", ask_quit_, "关闭启动器？",
        gui_open ? "仿真界面还开着。关闭启动器会同时关闭仿真界面（界面里没保存的配置会丢失）、CarSim 服务和云端连接。"
                 : "关闭启动器会断开云端连接，并关闭 CarSim 服务。",
        "modal:quit", "全部关闭", [this] {
          Stop("启动器关闭", false);
          quit_ = true;
        });
  modal("##stop", ask_stop_, "停止仿真？",
        "会关闭仿真界面（界面里没保存的配置会丢失）、CarSim 服务，并断开云端连接。", "modal:stop", "停止",
        [this] { Stop("已停止", false); });
}

void Launcher::AskQuit() {
  if (Running() && phase_ != Phase::Stopping) ask_quit_ = true;
  else quit_ = true;
}

// ============================================================================
// --tour: real clicks and screenshots, for the tests
// ============================================================================
void Launcher::BuildTour(const std::string& script) {
  std::error_code ec;
  fs::create_directories(fs::u8path(tour_dir_), ec);
  auto settled = [this] { return phase_ != Phase::Checking && !pip_; };
  auto click = [this](const char* t) { return [this, t] { click_target_ = t; }; };
  auto none = [] {};
  if (script == "main") {
    // Auto start; ready; the GUI (its own --tour through --gui-arg, or the stop button) ends
    // it; start again; ready; the close button asks; close everything.
    tour_ = {
        {none, [this] { return phase_ == Phase::Starting || phase_ == Phase::Ready || phase_ == Phase::Failed; }, "01_starting"},
        {none, [this] { return (phase_ == Phase::Ready && gui_.st == St::Ok) || phase_ == Phase::Failed; }, "02_ready"},
        {[this] { if (gui_extra_.empty()) click_target_ = "banner:stop"; },
         [this] { return gui_extra_.empty() ? ask_stop_ || phase_ == Phase::Stopped : phase_ == Phase::Stopped; }, "03_stop_or_gui_closed"},
        {[this] { if (phase_ != Phase::Stopped) click_target_ = "modal:stop"; },
         [this] { return phase_ == Phase::Stopped || phase_ == Phase::Failed; }, "04_stopped"},
        {[this] { gui_extra_.clear(); click_target_ = "banner:start"; },
         [this] { return (phase_ == Phase::Ready && gui_.st == St::Ok) || phase_ == Phase::Failed; }, "05_ready_again"},
        {[this] { AskQuit(); }, [this] { return ask_quit_; }, "06_quit_asks"},
        {click("modal:quit"), [this] { return quit_; }, ""},
    };
  } else if (script == "snap") {
    // One look at the state it settles in (checks, or a failure), then quit.
    tour_ = {{none, [this, settled] { return settled() && phase_ != Phase::Connecting && phase_ != Phase::Starting && phase_ != Phase::Stopping; }, "snap"},
             {[this] { quit_ = true; }, [] { return true; }, ""}};
  } else if (script == "light") {
    tour_ = {{none, [this, settled] { return settled(); }, ""},
             {click("hdr:theme"), [this] { return !dark_; }, "light"},
             {click("hdr:theme"), [this] { return dark_; }, ""},
             {[this] { quit_ = true; }, [] { return true; }, ""}};
  } else if (script == "numpy") {
    // No numpy: the button installs it, the checks run again.
    tour_ = {{none, [this, settled] { return settled(); }, "numpy_missing"},
             {click("check:numpy"), [this] { return pip_ != nullptr; }, "numpy_installing"},
             {none, [this, settled] { return settled() && checks_[kNumpy].st != St::Busy; }, "numpy_done"},
             {[this] { quit_ = true; }, [] { return true; }, ""}};
  } else if (script == "oldtunnel") {
    tour_ = {{none, [this, settled] { return settled(); }, "port_busy"},
             {click("check:kill_old"), [this] { return phase_ == Phase::Checking || checks_[kPorts].st == St::Ok; }, ""},
             {none, [this, settled] { return settled(); }, "port_free"},
             {[this] { quit_ = true; }, [] { return true; }, ""}};
  } else if (script == "pick") {
    // "选择…" with a test path (plat::SetTestPick), then back to automatic.
    tour_ = {{none, [this, settled] { return settled(); }, ""},
             {click("set:py_pick"), [this, settled] { return !python_setting_.empty(); }, ""},
             {none, [this, settled] { return settled(); }, "picked"},
             {click("set:py_auto"), [this] { return python_setting_.empty(); }, ""},
             {none, [this, settled] { return settled(); }, "auto_again"},
             {[this] { quit_ = true; }, [] { return true; }, ""}};
  } else if (script == "reconnect") {
    // Ready; the ssh process dies (the test kills it): reconnects by itself, ready again.
    tour_ = {{none, [this] { return (phase_ == Phase::Ready && gui_.st == St::Ok) || phase_ == Phase::Failed; }, "r1_ready"},
             {none, [this] { return reconnects_ > 0 || phase_ == Phase::Failed; }, "r2_reconnecting"},
             {none, [this] { return (phase_ == Phase::Ready && server_ready_ && cloud_.st == St::Ok) || phase_ == Phase::Failed; }, "r3_back"},
             {click("banner:stop"), [this] { return ask_stop_; }, ""},
             {click("modal:stop"), [this] { return phase_ == Phase::Stopped; }, "r4_stopped"},
             {[this] { quit_ = true; }, [] { return true; }, ""}};
  }
}

void Launcher::TourTick() {
  if (tour_dir_.empty() || tour_i_ >= tour_.size()) return;
  TourStep& st = tour_[tour_i_];
  if (!tour_started_) {
    if (frame_ < 10) return;
    st.action();
    tour_started_ = true;
    tour_since_ready_ = tour_frames_ = 0;
    return;
  }
  ++tour_frames_;
  if (st.ready()) {
    if (++tour_since_ready_ == 12) {
      if (!st.shot.empty()) shot_name_ = st.shot;
      Log(kLauncher, kInfo, "TOUR step " + std::to_string(tour_i_) + " ok" + (st.shot.empty() ? "" : " -> " + st.shot));
      ++tour_i_;
      tour_started_ = false;
    }
  } else if (tour_frames_ > 60 * 600) {
    Log(kLauncher, kErr, "TOUR step " + std::to_string(tour_i_) + " TIMEOUT");
    shot_name_ = (st.shot.empty() ? "step" + std::to_string(tour_i_) : st.shot) + "_TIMEOUT";
    ++tour_i_;
    tour_started_ = false;
  }
}

void Launcher::BeforeNewFrame() {
  ui::SetTourTarget(click_target_);
  if (click_target_.empty()) return;
  ImVec2 c;
  if (!ui::FindTarget(click_target_, &c) || !ui::TargetShown(click_target_)) {
    if (++click_phase_ > 600) {
      Log(kLauncher, kErr, "TOUR click target missing: " + click_target_);
      click_target_.clear();
      click_phase_ = 0;
    }
    return;
  }
  if (click_phase_ > 6) click_phase_ = 0;  // (counted frames while waiting for the target)
  if (click_phase_ <= 2 && (c.x != click_last_x_ || c.y != click_last_y_)) click_phase_ = 0;
  click_last_x_ = c.x;
  click_last_y_ = c.y;
  ImGuiIO& io = ImGui::GetIO();
  if (click_phase_ < 6) io.AddMousePosEvent(c.x, c.y);
  switch (click_phase_++) {
    case 2: io.AddMouseButtonEvent(ImGuiMouseButton_Left, true); break;
    case 4: io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); break;
    case 6:
      io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
      Log(kLauncher, kInfo, "TOUR clicked " + click_target_);
      click_target_.clear();
      click_phase_ = 0;
      break;
    default: break;
  }
}

void Launcher::AfterRender(int fb_w, int fb_h) {
  if (shot_name_.empty()) return;
  std::vector<unsigned char> px(static_cast<size_t>(fb_w) * static_cast<size_t>(fb_h) * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, fb_w, fb_h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
  const std::string path = U8(fs::u8path(tour_dir_) / (shot_name_ + ".ppm"));
  if (FILE* f = OpenFile(path, "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", fb_w, fb_h);
    for (int y = fb_h - 1; y >= 0; --y)
      std::fwrite(&px[static_cast<size_t>(y) * static_cast<size_t>(fb_w) * 3], 1, static_cast<size_t>(fb_w) * 3, f);
    std::fclose(f);
  }
  shot_name_.clear();
}

void Launcher::WriteState() {
  static const char* kSt[] = {"idle", "busy", "ok", "warn", "fail"};
  static const char* kPhase[] = {"checking", "idle", "connecting", "starting", "ready", "stopping", "stopped", "failed"};
  json j;
  j["phase"] = kPhase[static_cast<int>(phase_)];
  j["fail"] = {{"title", fail_title_}, {"hint", fail_hint_}};
  for (const auto& c : checks_) j["checks"].push_back({{"name", c.name}, {"st", kSt[static_cast<int>(c.st)]}, {"detail", c.detail}, {"hint", c.hint}});
  j["python_argv"] = py_argv_;
  j["python_setting"] = python_setting_;
  j["reconnects"] = reconnects_;
  j["tour_steps_done"] = tour_i_;
  j["tour_steps"] = tour_.size();
  for (const auto& l : log_) j["log"].push_back({{"src", l.src}, {"lvl", l.lvl}, {"text", l.text}});
  std::ofstream f(fs::u8path(tour_dir_) / "launcher_state.json");
  f << j.dump(1) << "\n";
}
