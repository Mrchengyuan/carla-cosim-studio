#include "imgui_internal.h"
#include "view_modes.h"
#include "app.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>

#include "config_file.h"
#include "imgui.h"
#include "jpeg_decode.h"
#include "ui_kit.h"

#if defined(_WIN32)
#include <windows.h>
#endif
#include <GLFW/glfw3.h>

namespace fs = std::filesystem;

// --------------------------------------------------------------------------
// shared helpers
// --------------------------------------------------------------------------
namespace appui {

static int StrResizeCb(ImGuiInputTextCallbackData* d) {
  if (d->EventFlag == ImGuiInputTextFlags_CallbackResize) {
    auto* s = static_cast<std::string*>(d->UserData);
    s->resize(static_cast<size_t>(d->BufTextLen));
    d->Buf = &(*s)[0];
  }
  return 0;
}

bool InputStr(const char* label, std::string& s, int flags) {
  if (s.capacity() == 0) s.reserve(16);
  return ImGui::InputText(label, &s[0], s.capacity() + 1, flags | ImGuiInputTextFlags_CallbackResize,
                          StrResizeCb, &s);
}

bool InputStrMultiline(const char* label, std::string& s, float w, float h) {
  if (s.capacity() == 0) s.reserve(16);
  return ImGui::InputTextMultiline(label, &s[0], s.capacity() + 1, ImVec2(w, h),
                                   ImGuiInputTextFlags_CallbackResize, StrResizeCb, &s);
}

bool ComboStr(const char* label, std::string& value, const std::vector<std::string>& items,
              const std::vector<std::string>* shown) {
  bool changed = false;
  size_t cur = static_cast<size_t>(std::find(items.begin(), items.end(), value) - items.begin());
  const char* preview = cur < items.size() ? (shown ? (*shown)[cur].c_str() : items[cur].c_str()) : value.c_str();
  // Tour click targets: "combo:<label without ##>" and, while open, "combo:<label>:<item>".
  const std::string target = std::string("combo:") + (std::string(label).rfind("##", 0) == 0 ? label + 2 : label);
  if (ImGui::BeginCombo(label, preview)) {
    for (size_t i = 0; i < items.size(); ++i) {
      const bool sel = (i == cur);
      if (ImGui::Selectable(shown ? (*shown)[i].c_str() : items[i].c_str(), sel)) {
        value = items[i];
        changed = true;
      }
      ui::RecordTarget(target + ":" + items[i]);
      if (sel) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  } else {
    ui::RecordTarget(target);  // (while open, the last item is the popup)
  }
  return changed;
}

std::string Fmt(const char* fmt, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return buf;
}

}  // namespace appui

using appui::Fmt;

namespace {

std::string NowStr() {
  std::time_t t = std::time(nullptr);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&t));
  return buf;
}

bool Base64Decode(const std::string& in, std::vector<unsigned char>& out) {
  static int T[256];
  static bool init = false;
  if (!init) {
    for (int& t : T) t = -1;
    const char* a = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (int i = 0; i < 64; ++i) T[static_cast<unsigned char>(a[i])] = i;
    init = true;
  }
  out.clear();
  out.reserve(in.size() * 3 / 4);
  int val = 0, bits = -8;
  for (unsigned char c : in) {
    if (T[c] < 0) {
      if (c == '=') break;
      continue;
    }
    val = (val << 6) + T[c];
    bits += 6;
    if (bits >= 0) {
      out.push_back(static_cast<unsigned char>((val >> bits) & 0xFF));
      bits -= 8;
    }
  }
  return true;
}

// A live view frame's pixels (RGB, w x h): raw "rgb", or "jpeg" from a backend
// on a server (a remote GUI's hello asks for it). False: nothing to show.
bool FramePixels(const json& ev, std::vector<unsigned char>& px, int& w, int& h) {
  if (ev.contains("jpeg")) {
    std::vector<unsigned char> jpg;
    Base64Decode(ev.value("jpeg", std::string()), jpg);
    return DecodeJpeg(jpg, px, w, h);
  }
  Base64Decode(ev.value("rgb", std::string()), px);
  w = ev.value("w", 0);
  h = ev.value("h", 0);
  return static_cast<int>(px.size()) >= w * h * 3;
}

void PushHist(std::vector<float>& h, float v, size_t max) {
  h.push_back(v);
  if (h.size() > max) h.erase(h.begin(), h.begin() + static_cast<std::ptrdiff_t>(h.size() - max));
}

// Banner and log line of a run stopped by an error (cosim_state "error").
std::string RunErrorNote(const std::string& why) { return "运行出错已停止：" + why + "（详情见底部“输出”）"; }

// PROTOCOL of backend_server.py this GUI was built for (its "hello"): raise both
// together whenever a command, event or config field the GUI relies on changes.
constexpr int kBackendProtocol = 4;

const char* const kStoppingBackend = "正在停止后端（清理 CARLA 中的主车、传感器和交通）...";

}  // namespace

bool appui::Base64(const std::string& in, std::vector<unsigned char>& out) { return Base64Decode(in, out); }
bool appui::FramePixels(const json& ev, std::vector<unsigned char>& px, int& w, int& h) { return ::FramePixels(ev, px, w, h); }

struct TourStep {
  int panel;
  std::function<void()> action;
  std::function<bool()> ready;
  std::string shot;
};

// --------------------------------------------------------------------------
// lifecycle
// --------------------------------------------------------------------------
App::App() {
  // Minimal config so panels can render before the backend sends defaults.
  cfg_ = {{"carla", {{"vehicle", "vehicle.tesla.model3"}, {"spawn_index", 0}}}};
  saved_cfg_ = cfg_;
}

App::~App() {
  if (be_.Connected() && !Remote()) {  // (a backend on a server is stopped there)
    be_.Request("shutdown", json::object(), nullptr);
    for (int i = 0; i < 30 && plat::IsAlive(backend_proc_); ++i) glfwWaitEventsTimeout(0.1);
  }
  be_.Disconnect();
  if (backend_proc_.valid()) plat::Kill(backend_proc_);
  delete tour_;
}

void App::Init(int argc, char** argv) {
  plat::NetInit();
  const std::string exe_dir = plat::ExecutableDir();
  prefs_path_ = (fs::u8path(exe_dir) / "cosim_studio_prefs.json").u8string();

  std::string backend_dir;
  for (const char* rel : {"carsim_carla_bridge", "../carsim_carla_bridge", "../../carsim_carla_bridge",
                          "../../../carsim_carla_bridge"}) {
    fs::path p = fs::u8path(exe_dir) / rel / "backend_server.py";
    if (plat::FileExists(p.u8string())) {
      backend_dir = fs::weakly_canonical(p.parent_path()).u8string();
      break;
    }
  }
#ifdef _WIN32
  const char* py = "python";
#else
  const char* py = "python3";
#endif
  prefs_ = {{"python", py}, {"backend_dir", backend_dir}, {"backend_port", 57100},
            {"auto_start_backend", true}, {"remote_backend", false}, {"carla_host", "localhost"}, {"carla_port", 2000},
            {"last_config", ""}, {"dark_theme", true}};
  std::ifstream pf(fs::u8path(prefs_path_));
  if (pf) {
    json saved = json::parse(pf, nullptr, false);
    // A hand-edited value of the wrong type (e.g. "backend_port": "57100")
    // would throw on every start: keep the default for it.
    if (saved.is_object())
      for (auto& kv : saved.items()) {
        const json& def = prefs_.contains(kv.key()) ? prefs_[kv.key()] : json();
        if (def.is_null() || (def.is_number() && kv.value().is_number()) ||
            (def.is_string() && kv.value().is_string()) || (def.is_boolean() && kv.value().is_boolean()))
          prefs_[kv.key()] = kv.value();
      }
  }
  prefs_file_ = prefs_;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](std::string& dst) { if (i + 1 < argc) dst = argv[++i]; };
    std::string v;
    if (a == "--tour") next(tour_dir_);
    else if (a == "--hero") { next(tour_dir_); hero_spawns_ = hero_spawns_.empty() ? "3" : hero_spawns_; }
    else if (a == "--hero-spawns") next(hero_spawns_);
    else if (a == "--python") {
      next(v);
      // A relative path to a Python (not just "python3") means: from here. The
      // backend is started in the bridge directory, where it would not resolve.
      if (v.find_first_of("/\\") != std::string::npos && !fs::u8path(v).is_absolute())
        v = fs::absolute(fs::u8path(v)).u8string();
      prefs_["python"] = prefs_cli_["python"] = v;
    }
    else if (a == "--backend-dir") { next(v); prefs_["backend_dir"] = prefs_cli_["backend_dir"] = fs::absolute(fs::u8path(v)).u8string(); }
    else if (a == "--carla-port") { next(v); prefs_["carla_port"] = prefs_cli_["carla_port"] = std::atoi(v.c_str()); }
    else if (a == "--font") next(font_path_);
    else if (a == "--config") {
      next(v);
      // Like --python: from here (a path typed in the GUI is from the bridge directory).
      if (!v.empty() && !fs::u8path(v).is_absolute()) v = fs::absolute(fs::u8path(v)).lexically_normal().u8string();
      prefs_["last_config"] = prefs_cli_["last_config"] = v;
    }
    else if (a == "--light") prefs_["dark_theme"] = prefs_cli_["dark_theme"] = false;
    else if (a == "--auto-connect") auto_connect_ = true;
    else if (a == "--size" || a == "--scale") next(v);  // handled in main.cpp
  }
  dark_ = prefs_.value("dark_theme", true);

  std::string last = prefs_.value("last_config", std::string());
  if (!last.empty()) {
    if (plat::FileExists(UserPath(last))) LoadConfig(last);
    else Log("上次用的配置文件不见了：" + UserPath(last) + "，这次用默认设置", "warn");
  }
  if (prefs_.value("auto_start_backend", true)) StartBackend();
  if (!tour_dir_.empty()) {
    if (hero_spawns_.empty()) BuildTour(); else BuildHeroTour();
  } else {
    // The panel layout, next to the prefs (a tour always starts from the default one).
    layout_ini_ = (fs::u8path(prefs_path_).parent_path() / "cosim_studio_layout.ini").u8string();
    ImGui::GetIO().IniFilename = layout_ini_.c_str();
  }
}

void App::SavePrefs() {
  if (!tour_dir_.empty()) return;  // a scripted tour never changes the user's settings
  prefs_["dark_theme"] = dark_;
  // What the launcher passed (e.g. port 3000 for the modified CARLA, --light,
  // --config) is for this session only, unless the user changed it on the page.
  json out = prefs_;
  if (prefs_cli_.is_object())
    for (auto& kv : prefs_cli_.items())
      if (out.value(kv.key(), json()) == kv.value()) {
        if (prefs_file_.contains(kv.key())) out[kv.key()] = prefs_file_[kv.key()];
        else out.erase(kv.key());
      }
  std::string err;
  if (!cfgfile::WriteFileReplace(prefs_path_, out.dump(2, ' ', false, json::error_handler_t::replace), err))
    Log("无法保存界面设置到 " + prefs_path_ + "：" + err, "warn");
}

void App::Log(const std::string& msg, const std::string& level) {
  log_.push_back({level, NowStr(), msg});
  // The control algorithm's print() output (level "algo") keeps 800 lines of
  // its own: it never pushes the backend's messages out.
  const bool algo = level == "algo";
  auto same = [algo](const LogLine& l) { return (l.level == "algo") == algo; };
  if (std::count_if(log_.begin(), log_.end(), same) > 800) log_.erase(std::find_if(log_.begin(), log_.end(), same));
  log_scroll_ = true;
  if (level == "error") ++log_errors_;
  if (!tour_dir_.empty()) std::printf("[%s] %s\n", level.c_str(), msg.c_str());
}

void App::Call(const std::string& cmd, json args, std::function<void(const json&)> on_ok,
               const std::string& busy_text) {
  if (!busy_text.empty()) busy_ = busy_text;
  const bool clears_busy = !busy_text.empty();
  be_.Request(cmd, std::move(args), [this, on_ok, clears_busy](bool ok, const json& r, const std::string& err) {
    if (clears_busy) busy_.clear();
    if (!ok) {
      Log(err, "error");
      return;
    }
    if (on_ok) on_ok(r);
  });
}

const json* App::SelectedVehicleSpec() const {
  if (vehicle_sel_ < 0 || vehicle_sel_ >= static_cast<int>(vehicles_.size())) return nullptr;
  const json& v = vehicles_[static_cast<size_t>(vehicle_sel_)];
  return v.contains("spec") && v["spec"].is_object() ? &v["spec"] : nullptr;
}

// --------------------------------------------------------------------------
// backend / connection
// --------------------------------------------------------------------------
void App::StartBackend() {
  if (Remote() || plat::IsAlive(backend_proc_)) return;  // remote: the server starts it
  stop_since_ = -1;  // the old one is gone: nothing left to wait for
  if (backend_proc_.valid()) plat::Kill(backend_proc_, true);  // gone already: just release it
  backend_rejected_ = false;
  if (!backend_problem_.empty()) {
    // Started by hand after a crash or hang: clear what the old one left in CARLA.
    backend_problem_.clear();
    recover_connect_ = true;
  }
  std::string dir = prefs_.value("backend_dir", std::string());
  std::string script = (fs::u8path(dir) / "backend_server.py").u8string();
  if (dir.empty() || !plat::FileExists(script)) {
    Log("找不到 backend_server.py，请在“连接”页设置桥接目录", "error");
    return;
  }
  std::string log_path = (fs::u8path(dir) / "backend.log").u8string();
  // Keep the log of the previous backend: after a crash it holds the reason.
  std::error_code ec;
  if (fs::exists(fs::u8path(log_path), ec)) fs::rename(fs::u8path(log_path), fs::u8path(dir) / "backend.prev.log", ec);
  std::string err;
  if (!plat::Spawn({prefs_.value("python", std::string("python")), "-u", "backend_server.py", "--port",
                    std::to_string(prefs_.value("backend_port", 57100)), "--exit-with-client"},
                   dir, log_path, backend_proc_, err)) {
    Log("启动后端失败：" + err, "error");
    return;
  }
  Log("后端已启动");
}

void App::StopBackend() {
  if (Remote()) return;  // on the server: stopped there when the SSH session ends
  if (stop_since_ >= 0) return;  // already stopping
  // The backend cleans CARLA up and exits by itself; the window keeps drawing
  // meanwhile (StopBackendPoll ends it if it does not).
  stop_asked_ = be_.Connected();
  if (stop_asked_) be_.Request("shutdown", json::object(), nullptr);
  stop_termed_ = false;
  stop_since_ = ImGui::GetTime();
  busy_ = kStoppingBackend;
}

// Done once the backend has exited (one started elsewhere: once it closed the
// connection). Asked over the connection it has 3 s; then (at once when it
// could not be asked) SIGTERM, which cleans up too (POSIX; Windows has none);
// 5 s later it is killed.
void App::StopBackendPoll() {
  if (stop_since_ < 0) return;
  const double waited = ImGui::GetTime() - stop_since_, term_at = stop_asked_ ? 3.0 : 0.0;
  const bool gone = backend_proc_.valid() ? !plat::IsAlive(backend_proc_) : !be_.Connected();
  if (!gone && waited < term_at + 5.0) {
    if (waited >= term_at && !stop_termed_) {
      stop_termed_ = true;
      plat::Terminate(backend_proc_);
    }
    busy_ = kStoppingBackend;  // (the connection it closes clears it)
    return;
  }
  stop_since_ = -1;
  be_.Disconnect();
  if (backend_proc_.valid()) plat::Kill(backend_proc_, true);  // gone already, or it had its time
  OnEvent({{"event", "disconnected"}, {"quiet", true}});  // nothing of that backend's state is valid
  if (gone) Log("后端已停止，CARLA 中的主车、传感器和交通已清理");
  else Log("后端没有按时退出，已强制结束（CARLA 中可能还留有它的主车、传感器或交通）", "warn");
}

bool App::WantsQuit() {
  if (!quit_) return false;
  if (stop_since_ < 0 && (be_.Connected() || plat::IsAlive(backend_proc_))) StopBackend();
  return stop_since_ < 0;
}

void App::RestartBackend() {
  // A hung backend cannot clean up: have it print every thread's stack into
  // its log (kept as backend.prev.log), stop it hard, start a fresh one and
  // reconnect, removing what the old one left in CARLA.
  if (Remote()) {
    // On the server: asked over the connection, it writes the stacks into its
    // log and exits; the session script there starts a fresh one, which the
    // GUI reconnects to (see FrameBody).
    if (be_.Connected()) be_.Request("restart_backend", json::object(), nullptr);
    for (int i = 0; i < 20 && be_.Connected(); ++i) glfwWaitEventsTimeout(0.1);
  } else if (plat::IsAlive(backend_proc_)) {
    // Windows has no SIGUSR1: there the backend's socket thread prints them
    // on a "dump_stacks" line (the worker, which may be what hangs, is not needed).
    if (!plat::DumpStacks(backend_proc_) && be_.Connected()) be_.Request("dump_stacks", json::object(), nullptr);
    for (int i = 0; i < 5; ++i) glfwWaitEventsTimeout(0.1);
  }
  stop_since_ = -1;  // (a 停止后端 still waiting: this ends it)
  be_.Disconnect();
  plat::Kill(backend_proc_, true);
  OnEvent({{"event", "disconnected"}, {"quiet", true}});  // same reset as a lost connection
  backend_problem_.clear();
  Log("正在重启后端 ...", "warn");
  StartBackend();
  recover_connect_ = true;
}

void App::ConnectBackend(bool quiet) {
  std::string err;
  // A quiet retry while the backend starts: a short time-out (a backend
  // listening on this machine answers at once); on Windows every refused try
  // would otherwise stall the window for all of it.
  if (!be_.Connect("127.0.0.1", prefs_.value("backend_port", 57100), err, quiet ? 50 : 300)) {
    if (!quiet) Log(err, "error");
    return;
  }
  // Remote: the SSH tunnel accepts the connection even while no backend listens
  // on the server (and closes it again): it counts once the backend answers
  // hello, which also asks for the live views as JPEG (a fraction of the bytes).
  const bool remote = Remote();
  hello_ok_ = false;
  if (!remote) Log("已连接后端");
  // A git pull updates the Python side at once, this program only when it is
  // rebuilt: say so instead of misbehaving silently.
  be_.Request("hello", remote ? json{{"jpeg", true}} : json::object(), [this, remote](bool ok, const json& r, const std::string& err) {
    if (ok) hello_ok_ = true;
    if (ok && remote) {
      Log("已连接后端");
      LoadBackendDefaults();
      be_.Request("carsim_service", json::object(), [this](bool ok2, const json& s, const std::string&) {
        if (ok2) carsim_service_ = s;
      });
    }
    // A lost connection fails it too; only an old backend does not know the command.
    if (!ok && err.find("未知命令") == std::string::npos) return;
    const int theirs = ok && r.is_object() && r.contains("protocol") && r["protocol"].is_number_integer()
                           ? r["protocol"].get<int>() : 0;
    if (theirs != kBackendProtocol)
      Log(Fmt("界面与后端的版本不一致（界面 %d，后端 %d）：请重新编译 CoSim Studio，或把桥接目录更新到与界面相同的版本",
              kBackendProtocol, theirs), "error");
  });
  if (!remote) LoadBackendDefaults();
}

void App::LoadBackendDefaults() {
  // Always start from the backend's full defaults and lay what we already have
  // (e.g. a loaded, possibly partial config file) on top, so every section the
  // pages read exists.
  Call("default_config", json::object(), [this](const json& r) {
    json cur = cfg_.is_object() ? cfg_ : json::object();
    const bool clean = !ConfigDirty();
    cfg_defaults_ = r;
    cfg_ = r;
    cfg_.merge_patch(cur);
    ConformConfig();
    if (clean) saved_cfg_ = cfg_;  // filling in the defaults is no change of the user's
    RefreshDisk();
  });
  Call("rig_presets", json::object(), [this](const json& r) { rig_presets_ = r; });
}

void App::ConnectCarla(bool recover) {
  if (!be_.Connected()) ConnectBackend();
  json args = {{"host", prefs_.value("carla_host", std::string("localhost"))},
               {"port", prefs_.value("carla_port", 2000)}, {"recover", recover}};
  Call("connect", args, [this](const json& r) {
    carla_connected_ = true;
    rig_converting_ = false;  // an old rig whose conversion failed: try again with this connection
    server_info_ = r;
    run_note_.clear();  // it was about the previous connection (e.g. "CARLA 服务器已退出")
    SetWorld(r);
    if (r.contains("cosim_state") && r["cosim_state"].is_string()) run_state_ = r["cosim_state"].get<std::string>();
    map_choice_ = r.value("map", std::string());
    SavePrefs();
    RefreshAfterMapChange();
    Call("list_maps", json::object(), [this](const json& m) { maps_ = m.get<std::vector<std::string>>(); });
    Call("list_weathers", json::object(), [this](const json& w) { weathers_ = w.get<std::vector<std::string>>(); });
    RefreshVehicles();
  }, "正在连接 CARLA ...");
}

void App::SetWorld(const json& r) {
  // The "run ended, ego parked" banner is about that ego: drop it once the
  // ego is gone or replaced (map switched, ego deleted or respawned).
  if (run_note_ego_ < 0 && r.is_object()) run_note_ego_ = r.value("ego_id", 0);  // a failed start: its ego
  if (!Running() && r.is_object() && r.value("ego_id", 0) != run_note_ego_) run_note_.clear();
  world_ = r.is_object() ? r : json::object();
  // Off after a run (the ego is respawned, then parked) or for a new ego: the backend knows.
  if (world_.contains("ego_autopilot") && world_["ego_autopilot"].is_boolean()) autopilot_ = world_["ego_autopilot"].get<bool>();
  // The backend tracks CARLA's recorder (it stops it on exit, reconnect, map change and replay).
  recording_ = world_.contains("recording") && world_["recording"].is_string() &&
               !world_["recording"].get<std::string>().empty();
}

void App::RefreshWorld() {
  Call("world_info", json::object(), [this](const json& r) {
    SetWorld(r);
    // The backend's run state is authoritative (e.g. after a reconnect).
    if (r.contains("cosim_state") && r["cosim_state"].is_string()) run_state_ = r["cosim_state"].get<std::string>();
    if (!weather_dirty_) weather_edit_ = r.value("weather", json::object());  // slider changes not applied yet stay
  });
}

void App::RefreshAfterMapChange() {
  traffic_count_ = json::object();  // the backend removed its traffic (connect, map change)
  RefreshWorld();
  RefreshSpawnPoints();
  RefreshActors();
  view_on_ = false;
}

void App::RefreshDisk() {
  if (!be_.Connected()) return;  // the config pages stay open without a backend
  std::string dir = cfg_.contains("collect") ? cfg_["collect"].value("out_dir", std::string("datasets"))
                                             : std::string(".");
  Call("disk_info", {{"path", dir}}, [this](const json& r) { disk_ = r; });
}

void App::LoadMap(const std::string& name, std::function<void()> then) {
  busy_ = "正在加载地图 " + name + " ...";
  be_.Request("load_map", {{"name", name}}, [this, then](bool ok, const json& r, const std::string& err) {
    busy_.clear();
    if (!ok) {
      Log(err, "error");
      // The backend removed the ego and the traffic before loading, and may
      // have followed CARLA to the map it is on now.
      if (carla_connected_) RefreshAfterMapChange();
      return;
    }
    SetWorld(r);
    map_choice_ = r.value("map", std::string());
    Log("地图已切换为 " + r.value("map", std::string()));
    RefreshAfterMapChange();
    if (then) then();
  });
}

void App::ApplyWeatherPreset(const std::string& preset) {
  Call("set_weather", {{"preset", preset}}, [this, preset](const json& r) {
    weather_edit_ = r;
    weather_dirty_ = false;
    Log("天气：" + preset);
  });
}

void App::ApplyWeatherParams() {
  Call("set_weather", {{"params", weather_edit_}}, [this](const json& r) {
    weather_edit_ = r;
    weather_dirty_ = false;
    Log("天气参数已应用");
  });
}

void App::ApplyWorldSettings() {
  const json& w = world_edit_.is_object() ? world_edit_ : world_;
  json a = {{"synchronous", w.value("synchronous", false)},
            {"frame_dt", w.value("frame_dt", 0.0)},
            {"no_rendering", w.value("no_rendering", false)},
            {"idle_tick", w.value("idle_tick", false)}};
  Call("world_settings", a, [this](const json& r) {
    SetWorld(r);
    world_edit_ = json();
    Log("仿真设置已应用");
  });
}

void App::RefreshVehicles() {
  Call("list_vehicles", json::object(), [this](const json& r) {
    vehicles_ = r;
    const std::string want = cfg_["carla"].value("vehicle", std::string());
    vehicle_sel_ = -1;  // the old index may be another model in this list
    for (size_t i = 0; i < vehicles_.size(); ++i)
      if (vehicles_[i].value("id", std::string()) == want) vehicle_sel_ = static_cast<int>(i);
    if (vehicle_sel_ < 0 && !want.empty() && !vehicles_.empty())
      Log("这个 CARLA 里没有配置中的车型 " + want + "，请在“车辆与视角”页重新选择", "warn");
  });
}

void App::FetchVehicleSpecs(bool all) {
  json args = json::object();
  if (!all) {
    json ids = json::array();
    const std::string cur = cfg_["carla"].value("vehicle", std::string("vehicle.tesla.model3"));
    for (const std::string& id : {cur, std::string("vehicle.audi.tt"), std::string("vehicle.lincoln.mkz_2020"),
                                  std::string("vehicle.mercedes.coupe_2020"), std::string("vehicle.nissan.patrol_2021")})
      ids.push_back(id);
    args["ids"] = ids;
  }
  Call("vehicle_specs", args, [this](const json&) {
    RefreshVehicles();
    Log("车型尺寸已测量");
  }, "正在测量车型尺寸 ...");
}

void App::RefreshSpawnPoints() {
  Call("list_spawn_points", json::object(), [this](const json& r) { spawn_points_ = r; });
}

void App::SpawnEgo() {
  if (vehicle_sel_ < 0 || vehicle_sel_ >= static_cast<int>(vehicles_.size())) {
    Log("请先在车型列表里选择一款车", "warn");
    return;
  }
  std::string bp = vehicles_[static_cast<size_t>(vehicle_sel_)]["id"];
  cfg_["carla"]["vehicle"] = bp;
  std::string color;
  if (ego_custom_color_)
    color = Fmt("%d,%d,%d", static_cast<int>(ego_color_[0] * 255), static_cast<int>(ego_color_[1] * 255),
                static_cast<int>(ego_color_[2] * 255));
  json a = {{"blueprint", bp}, {"spawn_index", cfg_["carla"].value("spawn_index", 0)}, {"color", color}};
  Call("spawn_ego", a, [this](const json&) {
    autopilot_ = false;
    RefreshWorld();
    RefreshActors();
    if (view_on_) StartView();
  }, "正在生成主车 ...");
}

void App::DestroyEgo() {
  Call("destroy_ego", json::object(), [this](const json&) {
    view_on_ = false;
    RefreshWorld();
    RefreshActors();
  });
}

void App::SetSpectator(const std::string& mode) {
  Call("spectator", {{"mode", mode}}, [this](const json& r) { world_["spectator_mode"] = r; });
}

void App::SpawnTraffic() {
  json a = {{"vehicles", traffic_vehicles_}, {"walkers", traffic_walkers_}, {"seed", traffic_seed_},
            {"safe", traffic_safe_}};
  Call("spawn_traffic", a, [this](const json& r) {
    traffic_count_ = r;
    RefreshActors();
  }, "正在生成交通 ...");
}

void App::ClearTraffic() {
  Call("clear_traffic", json::object(), [this](const json&) {
    traffic_count_ = json::object();
    RefreshActors();
    Log("交通已清除");
  });
}

void App::RefreshActors() {
  Call("list_actors", {{"filter", actor_filter_.empty() ? "*" : actor_filter_}},
       [this](const json& r) { actors_ = r; });
}

void App::StartRun() {
  if (vehicle_sel_ >= 0 && vehicle_sel_ < static_cast<int>(vehicles_.size()))
    cfg_["carla"]["vehicle"] = vehicles_[static_cast<size_t>(vehicle_sel_)]["id"];
  cfgfile::SyncRunDriver(cfg_);
  // The scene tab shows this run's data in the units it was started with.
  const json units = cfg_.contains("carsim") ? cfg_["carsim"].value("units", json::object()) : json::object();
  busy_ = "正在启动 ...";
  starting_ = true;
  // Remote: real CarSim runs in the CarSim service on this computer, not beside the backend.
  json run_cfg = cfg_;
  if (Remote() && run_cfg.contains("carsim") && !run_cfg["carsim"].value("mock", false) &&
      !run_cfg["carsim"].value("chrono", false))  // the Chrono BMW runs beside the backend
    run_cfg["carsim"]["remote"] = true;
  be_.Request("cosim_start", {{"config", run_cfg}}, [this, units](bool ok, const json& r, const std::string& err) {
    busy_.clear();
    starting_ = false;
    if (!ok) {
      // The run did not start (e.g. an error in the control algorithm): say
      // why on the viewport and show the output.
      run_note_level_ = "error";
      run_note_ = "运行没有启动：" + err;
      Log(run_note_, "error");
      run_note_ego_ = -1;  // about the ego the failed start leaves (it respawns it), see SetWorld
      log_open_ = true;
      dock_tab_select_ = 2;
      if (carla_connected_) RefreshWorld();
      return;
    }
    // Only once the run has started: a refused start (e.g. a typo in the
    // controller) keeps the last run's curves, trail and scene to compare with.
    for (auto* h : {&h_t_, &h_speed_, &h_steer_fl_, &h_steer_fr_, &h_rt_, &h_thr_, &h_brk_, &h_u3_}) h->clear();
    h_dbg_.clear();
    for (auto& h : h_susp_) h.clear();
    trail_x_.clear();
    trail_y_.clear();
    last_tel_ = json::object();
    last_scene_ = json();
    draw_ = json::array();
    scene_hover_.clear();
    // Step 0's collect_stats arrives before this reply: keep this run's, drop the last run's.
    const std::string root = r.is_object() && r.contains("collect") && r["collect"].is_object()
                                 ? r["collect"].value("root", std::string())
                                 : std::string();
    if (root.empty() || collect_stats_.value("root", std::string()) != root) collect_stats_ = json::object();
    run_info_ = r.is_object() ? r : json::object();
    run_info_["units"] = units;
    RefreshWorld();
  });
}

void App::RunCommand(const std::string& cmd) { Call(cmd, json::object(), nullptr); }

void App::StartView(const json& mount, float fov) {
  if (!mount.is_null()) {
    view_rig_mount_ = mount;
    view_rig_fov_ = fov;
  } else {
    view_rig_sensor_.clear();
    view_rig_mount_ = json();
  }
  SendViews();
}

// A rig sensor's mount for a view: CarSim's vehicle frame relative to the
// reference point, or, until it is converted, an older config's CARLA mount
// (relative to the car, y right), which the backend takes as it is.
json App::RigMount(const json& s) const {
  json m = {{"x", s.value("x", 0.0)}, {"y", s.value("y", 0.0)}, {"z", s.value("z", 0.0)}, {"pitch", s.value("pitch", 0.0)},
            {"yaw", s.value("yaw", 0.0)}, {"roll", s.value("roll", 0.0)}};
  if (RigLegacy()) return m;
  m["frame"] = "carsim";
  m["ref"] = cfg_.contains("sync") ? cfg_["sync"].value("reference_point", json("front_axle")) : json("front_axle");
  return m;
}

// Source id -> backend view spec ({kind, mode | mount, attrs}).
json App::PaneSpec(const std::string& source, int w, int h) const {
  const bool bev = source == "lidar" || source == "radar";
  json v = {{"width", bev ? std::min(w, h) : w}, {"height", bev ? std::min(w, h) : h}, {"fps", 10}};
  if (source.rfind("cam:", 0) == 0) {
    v["kind"] = "rgb";
    v["mode"] = source.substr(4);
  } else if (source == "semantic" || source == "depth" || source == "instance") {
    v["kind"] = source;
    v["mode"] = "hood";
  } else if (bev) {
    v["kind"] = source;
  } else if (source.rfind("rig:", 0) == 0) {
    const std::string name = source.substr(4);
    for (const json& s : const_cast<App*>(this)->RigSensors()) {
      if (s.value("name", std::string()) != name) continue;
      const std::string t = s.value("type", std::string());
      if (t != "rgb" && t != "depth" && t != "semantic" && t != "instance" && t != "lidar" && t != "radar")
        return json();  // e.g. an IMU: nothing to show as an image
      v["kind"] = t;
      v["mount"] = RigMount(s);
      v["attrs"] = s.value("attributes", json::object());
      if (t == "lidar" || t == "radar") v["width"] = v["height"] = std::min(w, h);
      return v;
    }
    return json();  // sensor no longer in the rig
  } else {
    return json();
  }
  return v;
}

std::vector<std::pair<std::string, std::string>> App::ViewSources() {
  std::vector<std::pair<std::string, std::string>> out;
  for (const ViewModeDef& m : kViewModes) out.push_back({std::string("cam:") + m.id, std::string("相机 · ") + m.name});
  for (const auto& e : std::vector<std::pair<std::string, std::string>>{
           {"semantic", "语义分割（前视）"}, {"depth", "深度（前视）"}, {"instance", "实例分割（前视）"},
           {"lidar", "激光雷达点云（俯视）"}, {"radar", "毫米波雷达（俯视）"}})
    out.push_back(e);
  static const char* kTypes[][2] = {{"rgb", "相机"}, {"depth", "深度"}, {"semantic", "语义"}, {"instance", "实例"},
                                    {"lidar", "激光雷达"}, {"radar", "毫米波雷达"}};
  for (const json& s : RigSensors()) {
    const std::string t = s.value("type", std::string());
    for (const auto& k : kTypes)
      if (t == k[0]) out.push_back({"rig:" + s.value("name", std::string()), Fmt("套件 · %s（%s）", s.value("name", std::string()).c_str(), k[1])});
  }
  return out;
}

void App::SendViews() {
  static const int kRes[][2] = {{480, 270}, {640, 360}, {960, 540}, {1280, 720}};
  // Main pane at the chosen resolution in single / 1+3 layouts; everything
  // smaller when four panes share the viewport.
  const int mw = view_layout_ == 2 ? 640 : kRes[view_res_][0], mh = view_layout_ == 2 ? 360 : kRes[view_res_][1];
  const int sw = view_layout_ == 2 ? 640 : 480, sh = view_layout_ == 2 ? 360 : 270;
  json main = {{"id", "p0"}, {"kind", "rgb"}, {"mode", view_mode_}, {"width", mw}, {"height", mh}, {"fps", 15}};
  if (!view_rig_mount_.is_null()) {
    main["mount"] = view_rig_mount_;
    main["attrs"] = {{"fov", view_rig_fov_}};
  }
  json views = json::array({main});
  const int n = view_layout_ == 0 ? 1 : 4;
  for (int i = 1; i < n; ++i) {
    json v = PaneSpec(panes_[i].source, sw, sh);
    if (v.is_null()) continue;
    v["id"] = Fmt("p%d", i);
    views.push_back(v);
    panes_[i].frames = 0;
  }
  // view_on_ follows the "views_active" event, which comes before this reply:
  // setting it here would reopen views the user closed while the call was running.
  ++views_pending_;
  be_.Request("views_set", {{"views", views}}, [this](bool ok, const json&, const std::string& err) {
    --views_pending_;
    if (!ok) Log(err, "error");
  });
}

static void UploadRgb(unsigned int& tex, int w, int h, const std::vector<unsigned char>& px) {
  if (!tex) {
    GLuint t = 0;
    glGenTextures(1, &t);
    tex = t;
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F);  // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F);
  }
  glBindTexture(GL_TEXTURE_2D, tex);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, px.data());
}

void App::UploadViewTexture() {
  if (view_dirty_) {
    view_dirty_ = false;
    UploadRgb(view_tex_, view_w_, view_h_, view_pixels_);
  }
  for (int i = 1; i < 4; ++i) {
    if (!panes_[i].dirty) continue;
    panes_[i].dirty = false;
    UploadRgb(panes_[i].tex, panes_[i].w, panes_[i].h, panes_[i].px);
  }
  for (auto& p : ds_panes_) {
    if (!p.dirty) continue;
    p.dirty = false;
    UploadRgb(p.tex, p.w, p.h, p.px);
  }
}

// Keyboard driving: W/S or arrow keys = throttle / brake, A/D = steer. Inputs
// ramp like a real pedal and wheel instead of jumping to full scale.
void App::UpdateKeyboardDriving() {
  if (!cfg_.contains("drive") || run_state_ != "running") return;
  const json& dr = cfg_["drive"];
  const bool cosim = dr.value("dynamics", std::string()) == "cosim";
  const bool manual = cosim ? dr.value("cosim_driver", std::string()) == "manual"
                            : dr.value("carla_driver", std::string()) == "manual";
  if (!manual) return;
  ImGuiIO& io = ImGui::GetIO();
  const float dt = io.DeltaTime;
  const bool typing = io.WantTextInput;
  auto down = [&](ImGuiKey a, ImGuiKey b) { return !typing && (ImGui::IsKeyDown(a) || ImGui::IsKeyDown(b)); };
  const bool up = down(ImGuiKey_W, ImGuiKey_UpArrow), dn = down(ImGuiKey_S, ImGuiKey_DownArrow);
  const bool lf = down(ImGuiKey_A, ImGuiKey_LeftArrow), rt = down(ImGuiKey_D, ImGuiKey_RightArrow);
  auto ramp = [dt](float& v, float target, float rate) {
    v += std::max(-rate * dt, std::min(rate * dt, target - v));
  };
  ramp(kb_throttle_, up ? 1.0f : 0.0f, up ? 1.5f : 4.0f);
  ramp(kb_brake_, dn ? 1.0f : 0.0f, dn ? 3.0f : 5.0f);
  ramp(kb_steer_, lf ? -1.0f : (rt ? 1.0f : 0.0f), (lf || rt) ? 1.2f : 2.5f);
  const double now = ImGui::GetTime();
  if (now - kb_last_send_ > 0.05) {
    kb_last_send_ = now;
    be_.Request("manual_control", {{"throttle", kb_throttle_}, {"brake", kb_brake_}, {"steer", kb_steer_}}, nullptr);
  }
}

std::string App::UserPath(const std::string& path) const {
  const std::string dir = prefs_.value("backend_dir", std::string());
  if (path.empty() || dir.empty() || fs::u8path(path).is_absolute()) return path;
  return (fs::u8path(dir) / fs::u8path(path)).lexically_normal().u8string();
}

void App::ConformConfig() {
  if (const int fixed = cfgfile::ConformConfig(cfg_, cfg_defaults_))
    Log(Fmt("配置里有 %d 个值的类型不对（多半是手改过），已改正：写成文字的数字换成数字，其余换成默认值", fixed), "warn");
}

void App::LoadConfig(const std::string& user_path) {
  const std::string path = UserPath(user_path);
  std::ifstream f(fs::u8path(path));
  json j = f ? json::parse(f, nullptr, false) : json();
  if (!j.is_object()) {
    Log("无法读取配置：" + path, "error");
    return;
  }
  // Configs from before rigs were in CarSim's vehicle frame (sensors, no
  // "frame"): mark them; the backend converts them with the measured vehicle
  // once CARLA is connected (see Frame()), the same way as for a run or the
  // command line (settings.load_dict). A rig without sensors has nothing to convert.
  if (j.contains("rig") && j["rig"].is_object() && !j["rig"].contains("frame") && j["rig"].contains("sensors") &&
      j["rig"]["sensors"].is_array() && !j["rig"]["sensors"].empty())
    j["rig"]["frame"] = "carla";
  // Like settings.load_dict (run_cosim.py): the file over the defaults, not
  // over the config loaded before. (Without the defaults yet, ConnectBackend
  // lays them under.)
  cfg_ = cfgfile::FileOverDefaults(cfg_defaults_, cfg_, j);
  rig_converting_ = false;  // a conversion that failed for the previous config: try this one
  ConformConfig();
  saved_cfg_ = cfg_;
  if (carla_connected_) RefreshVehicles();  // a run uses the selected vehicle: select the file's
  cfg_path_ = path;
  prefs_["last_config"] = path;
  SavePrefs();
  Log("已载入配置 " + path);
}

void App::SaveConfig(const std::string& user_path) {
  const std::string path = UserPath(user_path);
  cfgfile::ForSave(cfg_, prefs_.value("carla_host", std::string("localhost")), prefs_.value("carla_port", 2000));
  std::string err;
  if (!cfgfile::WriteFileReplace(path, cfg_.dump(2, ' ', false, json::error_handler_t::replace), err)) {
    Log("无法写入 " + path + "：" + err, "error");
    return;
  }
  cfg_path_ = path;
  saved_cfg_ = cfg_;
  prefs_["last_config"] = path;
  SavePrefs();
  Log("配置已保存到 " + path);
}

// --------------------------------------------------------------------------
// events pushed by the backend
// --------------------------------------------------------------------------
void App::OnEvent(const json& ev) {
  const std::string type = ev.value("event", std::string());
  if (type == "log") {
    Log(ev.value("msg", std::string()), ev.value("level", std::string("info")));
    if (ev.value("rejected", false)) backend_rejected_ = true;  // it closes the connection now
  } else if (type == "cosim_state") {
    run_state_ = ev.value("state", std::string());
    // Say clearly why a run ended on its own; a parked car otherwise looks stuck.
    if (run_state_ == "finished" || run_state_ == "error") {
      const double dur = cfg_.contains("sync") ? cfg_["sync"].value("duration", 0.0) : 0.0;
      const double t = last_tel_.value("t", 0.0);
      if (run_state_ == "error" && starting_) {
        return;  // a failed start: the cosim_start reply says why ("运行没有启动")
      } else if (run_state_ == "error") {
        run_note_level_ = "error";
        run_note_ = RunErrorNote(ev.value("detail", std::string()));
      } else if (!ev.value("detail", std::string()).empty()) {
        const std::string why = ev.value("detail", std::string());
        run_note_level_ = "warn";
        run_note_ = "运行已结束：" + why + "，主车已停车。";
        if (why.rfind("达到设定的运行时长", 0) == 0) run_note_ += "要一直运行，把“驾驶模式 → 运行时长”设为 0。";
      } else if (dur > 0 && t >= dur - 0.25) {
        run_note_level_ = "warn";
        run_note_ = Fmt("运行已结束：达到设定的运行时长 %.0f s，主车已停车。要一直运行，把“驾驶模式 → 运行时长”设为 0。", dur);
      } else {
        run_note_level_ = "warn";
        run_note_ = "运行已结束（CarSim 到达结束时间或采集达到上限），主车已停车。";
      }
      run_note_ego_ = world_.value("ego_id", 0);
      Log(run_note_, run_note_level_);
    } else {
      run_note_.clear();
    }
    if (!Running()) {
      // The car is parked now: clear the live readouts so the viewport HUD
      // does not keep showing the last speed (the plots keep the history).
      last_tel_ = json::object();
      if (carla_connected_) {  // not after "carla_lost": that world is gone
        RefreshWorld();
        RefreshDisk();
      }
      if (panel_ == kPanelDataset && be_.Connected()) DatasetRefresh();  // a collection may have added a session
      kb_throttle_ = kb_brake_ = kb_steer_ = 0;
    }
  } else if (type == "telemetry") {
    last_tel_ = ev.value("data", json::object());
    if (last_tel_.contains("draw")) {  // new lines of the algorithm (self.draw): kept until the next ones
      if (last_tel_["draw"].is_array()) draw_ = std::move(last_tel_["draw"]);
      last_tel_.erase("draw");
    }
    if (last_tel_.contains("scene")) {  // big: keep it out of the per-frame readouts
      if (last_tel_["scene"].is_object()) last_scene_ = std::move(last_tel_["scene"]);
      last_tel_.erase("scene");
    }
    // Defence in depth: a number the backend could not send (NaN) arrives as
    // null; the readers below expect numbers.
    for (auto& kv : last_tel_.items()) {
      if (kv.value().is_null()) kv.value() = 0.0;
      if (kv.value().is_array())
        for (auto& e : kv.value()) if (e.is_null()) e = 0.0;
    }
    const json& d = last_tel_;
    PushHist(h_t_, d.value("t", 0.0f), kHist);
    if (d.contains("location") && d["location"].size() >= 2) {
      PushHist(trail_x_, static_cast<float>(appui::NumAt(d["location"], 0)), 6000);
      PushHist(trail_y_, static_cast<float>(appui::NumAt(d["location"], 1)), 6000);
    }
    PushHist(h_speed_, d.value("speed_kmh", 0.0f), kHist);
    PushHist(h_rt_, d.value("rt_factor", 0.0f), kHist);
    // The telemetry has CARLA's wheel angles (+ = right): shown + = left like CarSim's Steer_L1 / Steer_R1.
    const json st = d.value("wheel_steer", json::array());
    PushHist(h_steer_fl_, st.size() > 0 ? -static_cast<float>(appui::NumAt(st, 0)) : 0.0f, kHist);
    PushHist(h_steer_fr_, st.size() > 1 ? -static_cast<float>(appui::NumAt(st, 1)) : 0.0f, kHist);
    const json su = d.value("wheel_suspension_mm", json::array());
    for (size_t i = 0; i < 4; ++i) PushHist(h_susp_[i], su.size() > i ? static_cast<float>(appui::NumAt(su, i)) : 0.0f, kHist);
    const json a = d.value("action", json::array());
    PushHist(h_thr_, a.size() > 0 ? static_cast<float>(appui::NumAt(a, 0)) : 0.0f, kHist);
    PushHist(h_brk_, a.size() > 1 ? static_cast<float>(appui::NumAt(a, 1)) : 0.0f, kHist);
    PushHist(h_u3_, a.size() > 2 ? static_cast<float>(appui::NumAt(a, 2)) : 0.0f, kHist);
    // The algorithm's self.debug: a series per name (a new name starts with NaN before it; not given: NaN).
    const json dbg = d.value("debug", json::object());
    if (dbg.is_object())
      for (const auto& kv : dbg.items())
        if (h_dbg_.size() < 32 &&
            std::none_of(h_dbg_.begin(), h_dbg_.end(), [&](const auto& sr) { return sr.first == kv.key(); }))
          h_dbg_.push_back({kv.key(), std::vector<float>(h_t_.size() - 1, std::numeric_limits<float>::quiet_NaN())});
    for (auto& sr : h_dbg_) {
      const bool has = dbg.is_object() && dbg.contains(sr.first) && dbg[sr.first].is_number();
      PushHist(sr.second, has ? dbg[sr.first].get<float>() : std::numeric_limits<float>::quiet_NaN(), kHist);
    }
    // In .sim import order: only the default 3 are known to be [油门, 制动, 方向盘角].
    n_imports_ = static_cast<int>(a.size());
    imports_named_ = d.value("dynamics", std::string()) != "CarSim" || n_imports_ == 3;
  } else if (type == "collect_stats") {
    collect_stats_ = ev;
  } else if (type == "frame") {
    const std::string vid = ev.value("view", std::string("p0"));
    if (vid.size() == 2 && vid[1] >= '1' && vid[1] <= '3') {
      ViewPane& pane = panes_[vid[1] - '0'];
      if (view_on_ && FramePixels(ev, pane.px, pane.w, pane.h)) {
        pane.dirty = true;
        ++pane.frames;
      }
    } else if (view_on_ && FramePixels(ev, view_pixels_, view_w_, view_h_)) {
      view_dirty_ = true;
      ++view_frames_;
    }
  } else if (type == "progress") {
    const int done = ev.value("done", 0), total = ev.value("total", 1);
    if (done < total) busy_ = Fmt("正在测量车型尺寸 %d/%d", done + 1, total);
  } else if (type == "busy") {
    busy_task_ = ev.value("task", std::string());
    busy_secs_ = ev.value("seconds", 0.0);
    busy_carla_gone_ = ev.value("carla_gone", false);
    busy_where_ = ev.value("where", std::string());
    busy_seen_ = ImGui::GetTime();
  } else if (type == "carla_lost") {
    // The CARLA server is gone: nothing of that world exists any more.
    carla_connected_ = false;
    view_on_ = false;
    recording_ = false;  // the recorder runs inside CARLA
    world_ = json::object();
    world_edit_ = json();
    weather_dirty_ = false;
    last_tel_ = json::object();
    run_note_level_ = "error";
    run_note_ego_ = 0;
    run_note_ = ev.value("reason", std::string()) + "。重新启动 CARLA 后点“连接”。";
  } else if (type == "ego_changed") {
    RefreshWorld();  // a replay moved on to the next recorded ego
  } else if (type == "ego_lost") {
    // CARLA removed the ego by itself (e.g. it fell off the map): say so, and
    // keep saying it until there is a new ego.
    world_["ego_id"] = 0;
    run_note_ego_ = 0;
    if (run_note_.empty()) {
      run_note_level_ = "warn";
      run_note_ = ev.value("reason", std::string()) + "。请重新生成主车。";
    }
  } else if (type == "views_active") {
    // The backend says which live views exist (e.g. none after a failed respawn).
    view_on_ = !ev.value("ids", json::array()).empty();
    view_frames_ = 0;  // a new set of views: count its own frames
  } else if (type == "export_progress") {
    ds_export_done_ = ev.value("done", 0);
    ds_export_total_ = ev.value("total", 0);
  } else if (type == "export_done") {
    ds_exporting_ = false;
    ds_export_result_ = ev;
    if (ev.value("ok", false)) {
      const json res = ev.value("result", json::object());  // (const json: never operator[] a missing key)
      Log(Fmt("导出完成：%s", res.value("out", std::string()).c_str()));
      if (res.contains("warning")) Log(res.value("warning", std::string()), "warn");
    }
    else
      Log("导出失败：" + ev.value("error", std::string()), "error");
  } else if (type == "carsim_service") {
    carsim_service_ = ev;
  } else if (type == "disconnected") {
    carsim_service_ = json::object();
    starting_ = false;
    // Nothing the backend owned is valid any more: never stay "running" or busy.
    carla_connected_ = false;
    if (Running()) run_state_ = "error";
    world_ = json::object();
    traffic_count_ = json::object();
    busy_.clear();
    rig_converting_ = false;
    view_on_ = false;
    recording_ = false;  // a backend that exits stops it; the next connect reports the state
    ds_play_ = false;
    ds_pending_ = 0;
    ds_dirty_ = false;
    ds_exporting_ = false;
    ds_sessions_ = json();  // listed again by the next backend
    disk_ = json::object();  // that backend's free space
    world_edit_ = json();
    weather_dirty_ = false;
    last_tel_ = json::object();
    busy_task_.clear();
    // (Not the one 停止后端 ended; remote: nor a tunnel with no backend behind it yet.)
    if (!ev.contains("quiet") && stop_since_ < 0 && (hello_ok_ || !Remote())) Log("与后端的连接断开了", "error");
    hello_ok_ = false;
  }
}

// --------------------------------------------------------------------------
// --tour: scripted walk through every panel, saving screenshots
// --------------------------------------------------------------------------
void App::BuildTour() {
  fs::create_directories(fs::u8path(tour_dir_));
  auto idle = [this] { return busy_.empty() && be_.PendingCount() == 0; };
  const std::string ds = (fs::u8path(tour_dir_) / "tour_dataset").u8string();
  const fs::path dir = fs::absolute(fs::u8path(tour_dir_));  // the backend resolves relative paths in its own folder
  const std::string cfg_file = (dir / "tour_config.json").u8string(), bad_ctrl = (dir / "tour_bad_controller.py").u8string();
  const std::string print_ctrl = (dir / "tour_print_controller.py").u8string();
  static const std::string kPrinted = "tour: control() 第 3 次调用";
  auto printed = [this] {  // the last line of the algorithm's own output is the one the tour controller printed
    const auto l = std::find_if(log_.rbegin(), log_.rend(), [](const LogLine& x) { return x.level == "algo"; });
    return l != log_.rend() && l->text == kPrinted;
  };
  tour_ = new std::vector<TourStep>{
      // (Remote prefs: the backend on the server, started there; through a tunnel it is there once it answers.)
      {kPanelConnect, [this] { ConnectBackend(); }, [this] { return be_.Connected() && (hello_ok_ || !Remote()); }, ""},
      {kPanelConnect, [this] { ConnectCarla(); }, [this, idle] { return carla_connected_ && idle(); }, "01_connect"},
      {kPanelWorld, [] {}, idle, "02_world"},
      {kPanelVehicle, [this] { FetchVehicleSpecs(false); }, idle, ""},
      {kPanelVehicle, [this] {
         for (size_t i = 0; i < vehicles_.size(); ++i)
           if (vehicles_[i]["id"] == "vehicle.tesla.model3") vehicle_sel_ = static_cast<int>(i);
         cfg_["carla"]["spawn_index"] = 3;
         SpawnEgo();
       }, [this, idle] { return idle() && world_.value("ego_id", 0) != 0; }, ""},
      {kPanelVehicle, [this] { SetSpectator("follow"); }, idle, "03_vehicle"},
      {kPanelRig, [this] { LoadRigPreset("nuscenes"); }, [this, idle] { return idle() && RigSensors().size() == 12; }, ""},
      {kPanelRig, [this] { rig_sel_ = 0; }, idle, "04_rig_nuscenes"},
      {kPanelView, [this] { view_mode_ = "wheel"; view_res_ = 2; StartView(); },
       [this, idle] { return idle() && view_frames_ > 5; }, "05_view_wheel"},
      // Multi-view, driven by real clicks: 2x2, change a pane's source, 1+3, back to single.
      {kPanelView, [this] { click_target_ = "layout:2"; },
       [this, idle] { return idle() && view_layout_ == 2 && panes_[1].frames > 3 && panes_[2].frames > 3 && panes_[3].frames > 3; },
       "05b_multiview_2x2"},
      {kPanelView, [this] { click_target_ = "pane:1"; }, [] { return true; }, ""},
      {kPanelView, [this] { click_target_ = "src:radar"; },
       [this, idle] { return idle() && panes_[1].source == "radar" && panes_[1].frames > 3; }, ""},
      {kPanelView, [this] { click_target_ = "layout:1"; },
       [this, idle] { return idle() && view_layout_ == 1 && panes_[3].frames > 3; }, "05c_multiview_1p3"},
      {kPanelView, [this] { click_target_ = "layout:0"; }, [this, idle] { return idle() && view_layout_ == 0; }, ""},
      // A rig camera picked on this page: its mount goes out in CarSim's vehicle frame (RigMount).
      {kPanelView, [this] { click_target_ = "combo:rigcam"; }, [] { return ui::TargetShown("combo:rigcam:cam_front_left"); }, ""},
      {kPanelView, [this] { click_target_ = "combo:rigcam:cam_front_left"; }, [this, idle] {
         return idle() && view_rig_sensor_ == "cam_front_left" && view_rig_mount_.is_object() &&
                view_rig_mount_.value("frame", std::string()) == "carsim" && view_rig_mount_.contains("ref") &&
                views_pending_ == 0 && view_frames_ > 3;
       }, "05d_rig_camera"},
      {kPanelView, [this] { click_target_ = "view:wheel"; },
       [this, idle] { return idle() && view_rig_sensor_.empty() && view_mode_ == "wheel" && views_pending_ == 0 && view_frames_ > 3; }, ""},
      // The six directions: the four new ones by clicks on the viewport's bar, pictures of each.
      {kPanelView, [this] { click_target_ = "view:front"; },
       [this, idle] { return idle() && view_mode_ == "front" && views_pending_ == 0 && view_frames_ > 3; }, "05e_view_front"},
      {kPanelView, [this] { click_target_ = "view:left"; },
       [this, idle] { return idle() && view_mode_ == "left" && views_pending_ == 0 && view_frames_ > 3; }, "05f_view_left"},
      {kPanelView, [this] { click_target_ = "view:right"; },
       [this, idle] { return idle() && view_mode_ == "right" && views_pending_ == 0 && view_frames_ > 3; }, "05g_view_right"},
      {kPanelView, [this] { click_target_ = "view:iso"; },
       [this, idle] { return idle() && view_mode_ == "iso" && views_pending_ == 0 && view_frames_ > 3; }, "05h_view_iso"},
      {kPanelView, [this] { click_target_ = "view:wheel"; },
       [this, idle] { return idle() && view_mode_ == "wheel" && views_pending_ == 0 && view_frames_ > 3; }, ""},
      // The Chrono BMW in place of CarSim, by clicks: ticking it unticks 模拟 CarSim and shows its speed; back.
      {kPanelCoSim, [this] { tour_kept_["carsim"] = cfg_["carsim"]; cfg_["carsim"]["mock"] = true; cfg_["carsim"]["chrono"] = false; },
       [] { return ui::TargetShown("cosim:chrono"); }, ""},
      {kPanelCoSim, [this] { click_target_ = "cosim:chrono"; }, [this] {
         return click_target_.empty() && cfg_["carsim"].value("chrono", false) && !cfg_["carsim"].value("mock", true) &&
                ui::TargetShown("cosim:chrono_speed");
       }, "08e_chrono"},
      {kPanelCoSim, [this] { click_target_ = "cosim:mock"; }, [this] {
         return click_target_.empty() && cfg_["carsim"].value("mock", false) && !cfg_["carsim"].value("chrono", true) &&
                !ui::TargetShown("cosim:chrono_speed");
       }, ""},
      {kPanelCoSim, [this] { cfg_["carsim"] = tour_kept_["carsim"]; }, [this] { return cfg_["carsim"] == tour_kept_["carsim"]; }, ""},
      {kPanelTraffic, [this] { traffic_vehicles_ = 12; traffic_walkers_ = 8; SpawnTraffic(); }, idle, "06_traffic"},
      {kPanelDrive, [this] {
         cfg_["drive"]["dynamics"] = "cosim";
         cfg_["drive"]["cosim_driver"] = "custom";
         cfg_["drive"]["target_speed_kmh"] = 35.0;
         cfg_["run"]["controller"]["path"] = "controllers/scene_controller.py";
         cfg_["run"]["controller"]["entry"] = "Controller";
       }, idle, "07_drive"},
      {kPanelScene, [this] { cfg_["scene"]["collision"] = "log"; }, idle, "07b_scene_config"},
      {kPanelScene, [this] {
         // The clicks below tick these: start from unticked whatever the config said.
         for (auto [arr, key] : {std::pair<json*, const char*>{&cfg_["scene"]["objects"], "Speed"},
                                 {&cfg_["scene"]["record"]["objects"], "model"}}) {
           json keep = json::array();
           for (const json& k : *arr) if (k != key) keep.push_back(k);
           *arr = keep;
         }
         click_target_ = "key:objects:Speed";
       }, [this] {
         for (const json& k : cfg_["scene"]["objects"]) if (k == "Speed") return true;
         return false;
       }, ""},
      {kPanelScene, [this] { click_target_ = "rec:objects:model"; }, [this] {
         for (const json& k : cfg_["scene"]["record"]["objects"]) if (k == "model") return true;
         return false;
       }, ""},
      {kPanelCoSim, [this] {
         cfg_["carsim"]["mock"] = true;
         cfg_["sync"]["duration"] = 0.0;  // stopped below with the toolbar button
         cfg_["run"]["log_path"] = "";
       }, idle, "08_cosim_config"},
      // “浏览…” for the .sim and the python_carsim_env folder (the system dialog answers with a test path);
      // an empty python_carsim_env folder is found from the .sim's folder.
      {kPanelCoSim, [this] {
         tour_kept_["carsim"] = cfg_["carsim"];
         const fs::path dir = fs::u8path(tour_dir_) / "tour_carsim_env";
         fs::create_directories(dir / "sim");
         std::ofstream(dir / "carsim_env.py") << "# tour\n";
         std::ofstream(dir / "sim" / "simfile.sim") << "VEHICLE_CODE tour\n";
         cfg_["carsim"]["mock"] = false;
         cfg_["carsim"]["repo_path"] = "";
         plat::SetTestPick((dir / "sim" / "simfile.sim").u8string());
         click_target_ = "pick:sim";
       }, [this] {
         return click_target_.empty() && cfg_["carsim"].value("sim_path", std::string()) ==
                (fs::u8path(tour_dir_) / "tour_carsim_env" / "sim" / "simfile.sim").u8string() && ui::TargetShown("cosim:repo_auto");
       }, "08a_repo_auto"},
      {kPanelCoSim, [this] {
         plat::SetTestPick((fs::u8path(tour_dir_) / "tour_carsim_env").u8string());
         click_target_ = "pick:repo";
       }, [this] {
         return click_target_.empty() && cfg_["carsim"].value("repo_path", std::string()) ==
                (fs::u8path(tour_dir_) / "tour_carsim_env").u8string() && !ui::TargetShown("cosim:repo_auto");
       }, ""},
      {kPanelCoSim, [this] {
         plat::SetTestPick("");
         cfg_["carsim"] = tour_kept_["carsim"];
       }, [this] { return cfg_["carsim"] == tour_kept_["carsim"]; }, ""},
      // "默认" asks first: 取消 keeps the config, 恢复默认 resets it (vehicle and spawn point kept); then put it back.
      {kPanelCoSim, [this] { tour_kept_["cfg"] = cfg_; click_target_ = "cfg:default"; },
       [this] { return ui::TargetShown("reset:cancel") && cfg_ == tour_kept_["cfg"]; }, "08b_reset_ask"},
      {kPanelCoSim, [this] { click_target_ = "reset:cancel"; },
       [this, idle] { return idle() && click_target_.empty() && !ui::TargetShown("reset:cancel") && cfg_ == tour_kept_["cfg"]; }, ""},
      {kPanelCoSim, [this] { click_target_ = "cfg:default"; }, [] { return ui::TargetShown("reset:ok"); }, ""},
      {kPanelCoSim, [this] { click_target_ = "reset:ok"; }, [this, idle] {
         return idle() && !ui::TargetShown("reset:ok") && !cfg_["carsim"].value("mock", true) &&
                cfg_["run"]["controller"] == cfg_defaults_["run"]["controller"] &&
                cfg_["carla"]["spawn_index"] == tour_kept_["cfg"]["carla"]["spawn_index"];
       }, "08c_reset_done"},
      {kPanelCoSim, [this] { cfg_ = tour_kept_["cfg"]; }, [this] { return cfg_ == tour_kept_["cfg"] && ui::TargetShown("保存*"); }, ""},
      // Unsaved changes: "保存*" saves (into the tour folder) and loses its "*", a change brings it back and
      // 文件 → 退出 asks; 保存并退出 that cannot save keeps the dialog open, 取消 closes it.
      {kPanelCoSim, [this, cfg_file] {
         tour_kept_["path"] = cfg_path_;
         tour_kept_["last_config"] = prefs_.value("last_config", std::string());
         cfg_path_ = cfg_file;
         click_target_ = "保存*";
       }, [this, cfg_file] {
         return !ConfigDirty() && ui::TargetShown("保存") && !ui::TargetShown("保存*") && fs::exists(fs::u8path(cfg_file));
       }, ""},
      {kPanelCoSim, [this] { click_target_ = "cosim:mock"; },
       [this] { return !cfg_["carsim"].value("mock", true) && ConfigDirty() && ui::TargetShown("保存*"); }, "08d_unsaved"},
      {kPanelCoSim, [this] { click_target_ = "menu:文件"; }, [] { return ui::TargetShown("menu:退出"); }, ""},
      {kPanelCoSim, [this] { if (ConfigDirty()) click_target_ = "menu:退出"; },  // (with nothing to save it quits)
       [this] { return !quit_ && !ui::TargetShown("menu:退出") && ui::TargetShown("quit:cancel"); }, "08e_quit_ask"},
      {kPanelCoSim, [this, dir] { cfg_path_ = (dir / "missing" / "tour_config.json").u8string(); click_target_ = "quit:save"; },
       [this] { return !quit_ && ConfigDirty() && ui::TargetShown("quit:save_failed") && ui::TargetShown("quit:cancel"); },
       "08f_quit_save_failed"},
      {kPanelCoSim, [this] { click_target_ = "quit:cancel"; }, [this] {
         return !quit_ && click_target_.empty() && !ui::TargetShown("quit:cancel") && ConfigDirty() && ui::TargetShown("保存*");
       }, ""},
      {kPanelCoSim, [this] {
         cfg_path_ = tour_kept_["path"].get<std::string>();
         prefs_["last_config"] = tour_kept_["last_config"];
         SavePrefs();
         click_target_ = "cosim:mock";
       }, [this] { return cfg_["carsim"].value("mock", false) && !ConfigDirty() && ui::TargetShown("保存"); }, ""},
      {kPanelCollect, [this, ds] {
         cfg_["collect"]["enabled"] = false;
         cfg_["collect"]["out_dir"] = ds;
       }, idle, "09_collect_config"},
      // Listed now, empty: the session collected below must show up later without 刷新.
      {kPanelDataset, [] {}, [this, idle] { return idle() && ds_sessions_.is_array() && ds_sessions_.empty(); }, ""},
      {kPanelView, [this] {
         view_mode_ = "chase";
         view_res_ = 2;
         view_layout_ = 1;
         panes_[1].source = "semantic";
         panes_[2].source = "lidar";
         panes_[3].source = "depth";
         StartView();
       }, idle, ""},
      // From here the toolbar and viewport are driven by real mouse clicks.
      {kPanelView, [this] { click_target_ = "运行"; },
       [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 5.0; }, "10_running_view"},
      {kPanelScene, [this] { click_target_ = "dock:scene"; },
       [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 6.0 && last_scene_.is_object(); }, "10b_running_scene"},
      {kPanelScene, [this] { click_target_ = "scene:moving_only"; },
       [this] { return scene_moving_only_ && last_tel_.value("t", 0.0) > 6.5; }, "10c_scene_moving_only"},
      {kPanelScene, [this] { click_target_ = "dock:vstate"; },
       [this] { return Running() && click_target_.empty() && ui::TargetShown("vstate:pose"); }, "10d_vehicle_state"},
      {kPanelDrive, [] {}, [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 7.0; }, "11_running_drive"},
      // A run keeps the config it started with: clicks on its settings change nothing, "默认" opens no dialog.
      {kPanelDrive, [this] { click_target_ = "run:duration+"; }, [this] {
         return Running() && click_target_.empty() && ui::TargetShown("run:duration+") && cfg_["sync"].value("duration", -1.0) == 0.0;
       }, ""},
      {kPanelCoSim, [this] { click_target_ = "cosim:mock"; }, [this] {
         return Running() && click_target_.empty() && ui::TargetShown("cosim:mock") && cfg_["carsim"].value("mock", false);
       }, "11a_running_locked"},
      {kPanelCoSim, [this] { click_target_ = "cfg:default"; }, [this] {
         return Running() && click_target_.empty() && ui::TargetShown("cfg:default") && !ui::TargetShown("reset:cancel");
       }, ""},
      {kPanelDrive, [this] { click_target_ = "暂停"; }, [this] { return run_state_ == "paused"; }, ""},
      {kPanelDrive, [this] { tour_mark_ = last_tel_.value("frame", 0); click_target_ = "单步"; },
       [this] { return run_state_ == "paused" && last_tel_.value("frame", 0) == tour_mark_ + 1; }, ""},
      {kPanelDrive, [this] { click_target_ = "继续"; }, [this] { return run_state_ == "running"; }, ""},
      {kPanelDrive, [this] { click_target_ = "停止"; },
       [this] { return run_state_ == "stopped" && last_tel_.empty(); }, "11b_stopped"},
      // A start that fails (the algorithm raises NameError on import): the banner says why, the output tab opens.
      {kPanelDrive, [this, bad_ctrl] {
         std::ofstream(fs::u8path(bad_ctrl)) << "x = undefined_name\n";
         cfg_["run"]["controller"]["path"] = bad_ctrl;
         log_open_ = false;
       }, idle, ""},
      {kPanelDrive, [this] { click_target_ = "运行"; }, [this, idle] {
         return idle() && !Running() && run_note_.rfind("运行没有启动：", 0) == 0 && run_note_.find("NameError") != std::string::npos &&
                ui::TargetShown("log:filter0");
       }, "11c_start_failed"},
      // What the algorithm prints goes to the 输出 page: the 算法 filter shows it, and only it.
      {kPanelDrive, [this, print_ctrl] {
         std::ofstream(fs::u8path(print_ctrl)) << "print('tour: 算法已加载')\nN = [0]\ndraw = None\n\n\ndef control(exports, t, dt):\n"
                                                  "    global draw, debug\n    N[0] += 1\n    if N[0] == 3:\n        print('" << kPrinted << "')\n"
                                                  "    debug = {'N': N[0], 't x 2': t * 2}\n"
                                                  "    draw = [{'points': [[0, 0], [10, 0.3], [20, 1.0]], 'color': [255, 60, 40], 'width': 0.1},\n"
                                                  "            {'points': [[0, 0], [10, -0.2], [20, -0.6]], 'color': [60, 90, 255]},\n"
                                                  "            {'points': [[20, 1.0]], 'color': [255, 215, 0], 'width': 0.12}]\n"
                                                  "    return [0.0, 0.0, 0.0]\n";
         cfg_["run"]["controller"]["path"] = print_ctrl;
         cfg_["run"]["controller"]["entry"] = "control";
         click_target_ = "运行";
       }, [this, printed] { return run_state_ == "running" && last_tel_.contains("ctrl_ms") && printed(); }, ""},
      {kPanelDrive, [this] { click_target_ = "log:filter3"; }, [this, printed] {
         return log_filter_ == 3 && printed() && ui::TargetShown("log:algo_last") && !ui::TargetShown("log:other");
       }, "11d_algo_output"},
      {kPanelDrive, [this] { click_target_ = "log:filter0"; }, [this] { return log_filter_ == 0; }, ""},
      // self.debug: the 曲线 panel's second row, a plot per value.
      {kPanelDrive, [this] { click_target_ = "dock:plots"; }, [this] {
         return Running() && h_dbg_.size() == 2 && h_dbg_[0].first == "N" && ui::TargetShown("plots:debug");
       }, "11e0_algo_debug"},
      {kPanelDrive, [this] { click_target_ = "dock:draw"; }, [this] {
         return Running() && draw_.size() == 3 && ui::TargetShown("draw:lines");
       }, "11e_algo_draw"},
      // Docking by real mouse drags: the 轨迹 tab out into the middle of the viewport: a floating window.
      {kPanelDrive, [this] {
         const ImGuiViewport* v = ImGui::GetMainViewport();
         drag_to_ = ImVec2(v->WorkPos.x + v->WorkSize.x * 0.45f, v->WorkPos.y + v->WorkSize.y * 0.35f);
         click_target_ = "dock:draw";
       }, [this] {
         // Out of the main dock space: floating (on its own, or in a floating dock node of its own).
         ImGuiWindow* w = ImGui::FindWindowByName("###draw");
         const bool out = w != nullptr && (w->DockNode == nullptr || ImGui::DockNodeGetRootNode(w->DockNode)->ID != dock_id_);
         return click_target_.empty() && out && w->WasActive && ui::TargetShown("draw:lines");  // (Active is cleared at each frame start)
       }, "11f_panel_floating"},
      {kPanelDrive, [this] { click_target_ = "menu:视图"; }, [] { return ui::TargetShown("menu:layout_reset"); }, ""},
      {kPanelDrive, [this] { click_target_ = "menu:layout_reset"; }, [this] {
         ImGuiWindow* w = ImGui::FindWindowByName("###draw");
         ImGuiWindow* l = ImGui::FindWindowByName("###log");
         return click_target_.empty() && w != nullptr && l != nullptr && w->DockIsActive && w->DockNode == l->DockNode;
       }, "11g_layout_reset"},
      {kPanelDrive, [this] { click_target_ = "dock:log"; }, [this] { return ui::TargetShown("log:filter0"); }, ""},
      {kPanelDrive, [this] { cfg_["run"]["controller"]["entry"] = "Controller"; click_target_ = "停止"; },
       [this] { return run_state_ == "stopped" && last_tel_.empty(); }, ""},
      {kPanelDrive, [this] { cfg_["run"]["controller"]["path"] = "controllers/scene_controller.py"; click_target_ = "dock:scene"; },
       [this] { return ui::TargetShown("scene:moving_only") && !ui::TargetShown("log:filter0"); }, ""},
      {kPanelDrive, [this] { click_target_ = "view:wheel"; },
       [this] { return view_on_ && view_mode_ == "wheel" && busy_.empty() && views_pending_ == 0 && view_frames_ > 3; }, ""},
      {kPanelDrive, [this] { click_target_ = "view:close"; }, [this] { return !view_on_; }, ""},
      {kPanelDrive, [this] { click_target_ = "viewport:action"; }, [this] { return view_on_; }, ""},
      // The 实时画面 page's button closes the view like the viewport's ×: it does not reopen by itself.
      {kPanelView, [this] { click_target_ = "view:page_toggle"; }, [this] { return !view_on_ && !view_auto_; }, ""},
      {kPanelView, [this] { click_target_ = "view:page_toggle"; }, [this] { return view_on_ && view_auto_; }, ""},
      {-1, [this] { click_target_ = "tab:场景"; }, [this] { return panel_ == kPanelScene; }, ""},
      {-1, [this] { click_target_ = "nav:数据采集"; }, [this] { return panel_ == kPanelCollect; }, ""},
      // Tiny capture (3 frames of a single small camera) to show live stats;
      // the tour output folder is deleted by the caller afterwards.
      {kPanelCollect, [this] {
         cfg_["drive"]["dynamics"] = "carla";
         cfg_["drive"]["carla_driver"] = "autopilot";
         cfg_["drive"]["tm_ignore_lights"] = true;
         cfg_["sync"]["frame_dt"] = 0.1;
         cfg_["sync"]["duration"] = 0.0;
         cfg_["rig"]["sensors"] = json::array({
             {{"name", "cam_front"}, {"type", "rgb"}, {"x", 0.1}, {"y", 0.0}, {"z", 1.6}, {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
              {"attributes", {{"image_size_x", 640}, {"image_size_y", 360}, {"fov", 90.0}}}, {"enabled", true}},
             {{"name", "lidar_top"}, {"type", "lidar"}, {"x", -1.4}, {"y", 0.0}, {"z", 1.9}, {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
              {"attributes", {{"channels", 16}, {"range", 50.0}, {"points_per_second", 100000}}}, {"enabled", true}}});
         cfg_["collect"]["session"] = "tour";
         cfg_["collect"]["enabled"] = true;
         cfg_["collect"]["max_frames"] = 3;
         cfg_["collect"]["max_gb"] = 0.1;
         StartRun();
       }, [this] {
         return !Running() && collect_stats_.value("frames", 0) >= 3 && run_note_.find("数据采集") != std::string::npos;
       }, "12_collect_done"},
      // Dataset browser and export, by clicks.
      {kPanelDataset, [this] { cfg_["collect"]["enabled"] = false; },
       [this, idle] { return idle() && ds_sessions_.is_array() && !ds_sessions_.empty(); }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:session:tour"; },
       [this, idle] { return idle() && !ds_root_.empty() && ds_pending_ == 0 && ds_panes_[0].frames > 0 && ds_panes_[1].frames > 0; }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:next"; }, [this, idle] { return idle() && ds_idx_ == 1 && ds_pending_ == 0; }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:play"; },
       [this, idle] { return idle() && !ds_play_ && ds_idx_ == DatasetFrameCount() - 1 && ds_pending_ == 0; }, "12b_dataset_browser"},
      {kPanelDataset, [this] { ds_fmt_ = 0; props_scroll_end_ = true; }, [] { return true; }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:export"; },
       [this, idle] { return idle() && !ds_exporting_ && ds_export_result_.value("ok", false); }, "12c_dataset_export"},
      // 运行对比: two made-up run records in the tour's folder, both picked by clicks, the key figures
      // side by side, the 对比 panel's plots.
      {kPanelRuns, [this, dir] {
         const fs::path root = dir / "tour_runs";
         for (int k = 0; k < 2; ++k) {
           const fs::path f = root / (k ? "20260101_120030_algo_b" : "20260101_120000_algo_a");
           fs::create_directories(f);
           std::ofstream(f / "run.json") << "{\"map\": \"Town04\", \"controller\": \"algo_" << (k ? "b" : "a")
                                         << ".py\", \"dynamics\": \"cosim\", \"carsim_mock\": true, \"t_start\": 0.0, \"t_end\": 9.9, "
                                         << "\"end\": \"finished\", \"kpi\": {\"lane_offset_rms\": " << (k ? 0.12 : 0.05)
                                         << ", \"collisions\": 0, \"distance\": " << (k ? 180 : 200) << "}, \"units\": {}}";
           std::ofstream m(f / "log.csv"), l(f / "log_lane.csv"), d(f / "log_debug.csv");
           m << "t,frame,ego_X,ego_Y,ego_Speed,u1,u2,u3\n";
           l << "t,frame,offset,heading_err\n";
           d << "t,frame,cost\n";
           for (int i = 0; i < 100; ++i) {
             const double t = i * 0.1;
             m << t << "," << i << "," << 20 * t << "," << (k ? 0.3 : 0.1) * std::sin(t) << "," << 72 - k * 5 * std::sin(t) << ","
               << 0.2 << ",0," << 5 * std::sin(t + k) << "\n";
             l << t << "," << i << "," << (k ? 0.12 : 0.05) * std::sin(2 * t) << ",0\n";
             d << t << "," << i << "," << 100 - 8 * t + k * 5 << "\n";
           }
         }
         runs_path_ = root.u8string();
         runs_list_ = json::object();
         click_target_ = "runs:refresh";
       }, [this] { return runs_list_.value("runs", json::array()).size() == 2; }, ""},
      {kPanelRuns, [this] { click_target_ = "runs:sel:0"; }, [this] { return runs_sel_.size() == 1; }, ""},
      {kPanelRuns, [this] { click_target_ = "runs:sel:1"; }, [this] {
         return runs_sel_.size() == 2 && runs_series_.size() >= 2 && runs_pending_ == 0 && ui::TargetShown("runs:kpi");
       }, "12d_runs_kpi"},
      {kPanelRuns, [this] { click_target_ = "runs:open"; }, [this] {
         return compare_open_ && ui::TargetShown("compare:plots") && compare_dbg_ == "cost";
       }, "12e_runs_compare"},
      {kPanelRuns, [this] { compare_open_ = false; runs_sel_.clear(); }, [this] { return !ui::TargetShown("compare:plots"); }, ""},
      {kPanelActors, [this] { ClearTraffic(); RefreshActors(); }, idle, "13_actors"},
      {kPanelRecorder, [] {}, idle, "14_recorder"},
      {kPanelTestScene, [this] { tour_kept_["spawn"] = cfg_["carla"]["spawn_index"]; click_target_ = "scn:town04"; },
       [this, idle] {
         const json st = world_.value("scenario_start", json());
         return idle() && world_.value("map", "") == "Town04_Opt" && st.is_object() &&
                cfg_["carla"]["spawn_index"] == st["index"] && ui::TargetShown("scn:start_ok");
       }, "14b_test_scene_town04"},
      {kPanelTestScene, [this] { cfg_["carla"]["spawn_index"] = tour_kept_["spawn"]; }, idle, ""},
      {kPanelWorld, [this] { LoadMap("Town03"); }, [this, idle] { return idle() && world_.value("map", "") == "Town03" && run_note_.empty(); },
       "15_map_town03"},  // the "ego parked" banner of the last run is gone with the ego
      {kPanelWorld, [this] { dark_ = false; theme_changed_ = true; }, idle, "16_light_theme"},
      {kPanelWorld, [this] { dark_ = true; theme_changed_ = true; LoadMap("Town10HD_Opt"); },
       [this, idle] { return idle() && world_.value("map", "") == "Town10HD_Opt"; }, ""},
  };
  // Remote prefs: the toolbar's "远程" badge and, for real CarSim, the CarSim service's
  // status on the CarSim page (by a click on its tab; no service needed: "未连接").
  if (Remote()) {
    tour_->push_back({kPanelWorld, [this] { tour_kept_["mock"] = cfg_["carsim"]["mock"]; cfg_["carsim"]["mock"] = false; },
                      idle, ""});
    tour_->push_back({-1, [this] { click_target_ = "tab:CarSim"; }, [this] {
                        return panel_ == kPanelCoSim && click_target_.empty() && ui::TargetShown("toolbar:remote") &&
                               ui::TargetShown("cosim:service");
                      }, "17_remote"});
    tour_->push_back({kPanelCoSim, [this] { cfg_["carsim"]["mock"] = tour_kept_["mock"]; }, idle, ""});
  }
  // 测试场景: a preset fills the table, a closure is added and deleted with real clicks.
  {
    std::vector<TourStep> scn = {
        {kPanelTestScene, [this] { tour_kept_["scenario"] = cfg_["scenario"]; click_target_ = "scn:preset:3"; },
         [this] { return cfg_["scenario"].value("enabled", false) && cfg_["scenario"]["closures"].size() == 3; }, ""},
        {kPanelTestScene, [this] { click_target_ = "scn:add"; },
         [this] { return cfg_["scenario"]["closures"].size() == 4 &&
                         cfg_["scenario"]["closures"][3].value("distance_m", 0.0) == 540.0; }, "07d_test_scene"},
        {kPanelTestScene, [this] { click_target_ = "scn:del:3"; },
         [this] { return cfg_["scenario"]["closures"].size() == 3; }, ""},
        {kPanelTestScene, [this] { cfg_["scenario"] = tour_kept_["scenario"]; },
         [this] { return cfg_["scenario"] == tour_kept_["scenario"]; }, ""},
    };
    auto at = std::find_if(tour_->begin(), tour_->end(), [](const TourStep& t) { return t.shot == "07b_scene_config"; });
    if (at != tour_->end()) tour_->insert(at + 1, scn.begin(), scn.end());
  }
  // “浏览…” for the algorithm file: the system's dialog (answering with a test path) when the backend is
  // here; with it on a server a list of the server's files, clicked through (up, into controllers and
  // examples, a file, 选择). Either way the entry comes from the file: ex4_function.py has control().
  {
    auto picked = [this] {
      const json& c = cfg_["run"]["controller"];
      return c.value("path", std::string()) == "controllers/examples/ex4_function.py" &&
             c.value("entry", std::string()) == "control";
    };
    std::vector<TourStep> algo;
    if (Remote()) {
      algo = {
          {kPanelDrive, [this] { tour_kept_["controller"] = cfg_["run"]["controller"]; click_target_ = "algo:browse"; },
           [] { return ui::TargetShown("algo:dir:examples"); }, ""},
          {kPanelDrive, [this] { click_target_ = "algo:up"; }, [] { return ui::TargetShown("algo:dir:controllers"); }, ""},
          {kPanelDrive, [this] { click_target_ = "algo:dir:controllers"; }, [] { return ui::TargetShown("algo:dir:examples"); }, ""},
          {kPanelDrive, [this] { click_target_ = "algo:dir:examples"; },
           [] { return ui::TargetShown("algo:file:ex4_function.py"); }, ""},
          {kPanelDrive, [this] { click_target_ = "algo:file:ex4_function.py"; },
           [this] { return algo_sel_.is_object() && algo_sel_.value("name", std::string()) == "ex4_function.py"; },
           "07a_algo_browser"},
          {kPanelDrive, [this] { click_target_ = "algo:choose"; },
           [this, picked] { return picked() && !ui::TargetShown("algo:choose"); }, ""},
      };
    } else {
      algo = {{kPanelDrive, [this] {
                 tour_kept_["controller"] = cfg_["run"]["controller"];
                 plat::SetTestPick(UserPath("controllers/examples/ex4_function.py"));
                 click_target_ = "algo:browse";
               }, [picked] { return picked(); }, "07a_algo_pick"}};
    }
    algo.push_back({kPanelDrive, [this] {
                      plat::SetTestPick("");
                      cfg_["run"]["controller"] = tour_kept_["controller"];
                    }, [this] { return cfg_["run"]["controller"] == tour_kept_["controller"]; }, ""});
    auto at = std::find_if(tour_->begin(), tour_->end(), [](const TourStep& t) { return t.shot == "07_drive"; });
    if (at != tour_->end()) tour_->insert(at + 1, algo.begin(), algo.end());
  }
}

// A CarSim co-simulation (mock CarSim, route following on the road) with
// traffic and the 1+3 multi-view, run from several spawn points; screenshots
// at a few moments of each run to pick a README cover from.
void App::BuildHeroTour() {
  fs::create_directories(fs::u8path(tour_dir_));
  auto idle = [this] { return busy_.empty() && be_.PendingCount() == 0; };
  const bool keep = prefs_cli_.contains("last_config");  // --config: keep that run's settings
  tour_ = new std::vector<TourStep>{
      {kPanelConnect, [this] { ConnectBackend(); }, [this] { return be_.Connected(); }, ""},
      {kPanelConnect, [this] { ConnectCarla(); }, [this, idle] { return carla_connected_ && idle(); }, ""},
      {kPanelWorld, [this] { ApplyWeatherPreset("ClearNoon"); }, idle, ""},
      {kPanelVehicle, [this] {
         for (size_t i = 0; i < vehicles_.size(); ++i)
           if (vehicles_[i]["id"] == "vehicle.tesla.model3") vehicle_sel_ = static_cast<int>(i);
         SpawnEgo();
       }, [this, idle] { return idle() && world_.value("ego_id", 0) != 0; }, ""},
      {kPanelTraffic, [this, keep] { if (!keep) { traffic_vehicles_ = 40; traffic_walkers_ = 20; SpawnTraffic(); } }, idle, ""},
      {kPanelView, [this, keep] {
         if (!keep) {  // --config given: its algorithm, CarSim and run settings (e.g. KMPPI on the Chrono BMW)
           cfg_["drive"]["dynamics"] = "cosim";
           cfg_["drive"]["cosim_driver"] = "route";
           cfg_["drive"]["target_speed_kmh"] = 35.0;
           cfg_["carsim"]["mock"] = true;
           cfg_["sync"]["duration"] = 0.0;
           cfg_["sync"]["frame_dt"] = 0.05;
         }
         cfg_["run"]["log_path"] = "";
         cfg_["collect"]["enabled"] = false;
         if (keep) dock_tab_select_ = 4;  // the 轨迹 tab: the algorithm's lines
         view_mode_ = "chase";
         view_res_ = 3;
         view_layout_ = 1;
         panes_[1].source = "semantic";
         panes_[2].source = "lidar";
         panes_[3].source = "depth";
         StartView();
       }, [this, idle] { return idle() && view_frames_ > 5; }, ""},
  };
  std::string list = hero_spawns_;
  for (size_t pos = 0; pos <= list.size();) {
    const size_t comma = std::min(list.find(',', pos), list.size());
    const int sp = std::atoi(list.substr(pos, comma - pos).c_str());
    pos = comma + 1;
    tour_->push_back({kPanelView, [this, sp, keep] { if (keep) dock_tab_select_ = 4; cfg_["carla"]["spawn_index"] = sp; StartRun(); },
                      [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 4.0; }, ""});
    for (int t : {7, 10, 13, 16})
      tour_->push_back({kPanelView, [] {},
                        [this, t] { return run_state_ == "running" && last_tel_.value("t", 0.0) > t && panes_[1].frames > 3; },
                        Fmt("hero_s%03d_t%02d", sp, t)});
    tour_->push_back({kPanelView, [this] { RunCommand("cosim_stop"); }, [this] { return !Running(); }, ""});
  }
}

void App::TourTick() {
  static bool started = false;
  static int since_ready = 0, frames_in_step = 0;
  if (tour_i_ >= tour_->size()) {
    if (++tour_wait_ > 30 && !quit_) {  // (the backend then stops while frames go on)
      Log("TOUR DONE");
      quit_ = true;
    }
    return;
  }
  TourStep& s = (*tour_)[tour_i_];
  if (s.panel >= 0) panel_ = s.panel;  // < 0: a click step that changes the page itself
  if (!started) {
    if (tour_i_ == 0 && frame_ % 20 != 0) return;
    s.action();
    started = true;
    since_ready = frames_in_step = 0;
    return;
  }
  ++frames_in_step;
  if (tour_i_ == 0 && !be_.Connected() && frames_in_step % 30 == 0) s.action();
  if (s.ready()) {
    if (++since_ready == 20) {
      if (!s.shot.empty()) shot_name_ = s.shot;
      Log("TOUR step " + std::to_string(tour_i_) + " ok" + (s.shot.empty() ? "" : " -> " + s.shot));
      ++tour_i_;
      started = false;
    }
  } else if (frames_in_step > 60 * 240) {
    Log("TOUR step " + std::to_string(tour_i_) + " TIMEOUT (click " + (click_target_.empty() ? "-" : click_target_) +
        ", busy " + (busy_.empty() ? "-" : busy_) + ", run " + run_state_ + ", view " + (view_on_ ? "on" : "off") + ")", "error");
    shot_name_ = (s.shot.empty() ? "step" + std::to_string(tour_i_) : s.shot) + "_TIMEOUT";
    ++tour_i_;
    started = false;
  }
}

void App::TourClick() {
  ui::SetTourTarget(click_target_);
  if (click_target_.empty()) return;
  ImGuiIO& dio = ImGui::GetIO();
  if (drag_to_.x >= 0 && drag_from_.x >= 0) {
    // A drag: pressed on the target (a dock tab), moved to drag_to_ over 30 frames, held, released
    // there. The tab moves with the mouse (it may stop being a target): from where it started.
    const int ph = click_phase_++;
    const float k = std::clamp((ph - 4) / 30.0f, 0.0f, 1.0f);
    dio.AddMousePosEvent(drag_from_.x + (drag_to_.x - drag_from_.x) * k, drag_from_.y + (drag_to_.y - drag_from_.y) * k);
    if (ph == 2) dio.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
    if (ph == 44) dio.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
    if (ph == 48) {
      dio.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
      click_target_.clear();
      drag_to_ = drag_from_ = ImVec2(-1.0f, -1.0f);
      click_phase_ = 0;
    }
    return;
  }
  ImVec2 c;
  if (!ui::FindTarget(click_target_, &c)) {
    if (++click_phase_ > 120) {
      Log("TOUR click target missing: " + click_target_, "error");
      click_target_.clear();
      click_phase_ = 0;
    }
    return;
  }
  // A target still moving (its panel scrolling it into view) would get the press in one
  // place and the release in another, which ImGui does not count as a click: wait until it holds still.
  if (click_phase_ <= 2 && drag_to_.x < 0 && (c.x != click_last_.x || c.y != click_last_.y)) click_phase_ = 0;
  click_last_ = c;
  if (drag_to_.x >= 0) {  // a drag starts here
    drag_from_ = c;
    dio.AddMousePosEvent(c.x, c.y);
    click_phase_ = 1;
    return;
  }
  ImGuiIO& io = ImGui::GetIO();
  // Every frame: with the window focused (e.g. on Windows) the GLFW backend
  // reports the real cursor each frame, which would move the press elsewhere.
  if (click_phase_ < 6) io.AddMousePosEvent(c.x, c.y);
  switch (click_phase_++) {
    case 2: io.AddMouseButtonEvent(ImGuiMouseButton_Left, true); break;
    case 4: io.AddMouseButtonEvent(ImGuiMouseButton_Left, false); break;
    case 6: io.AddMousePosEvent(-FLT_MAX, -FLT_MAX); click_target_.clear(); click_phase_ = 0; break;
    default: break;
  }
}

void App::AfterRender(int fb_w, int fb_h) {
  if (shot_name_.empty()) return;
  std::vector<unsigned char> px(static_cast<size_t>(fb_w) * static_cast<size_t>(fb_h) * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, fb_w, fb_h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
  std::string path = (fs::u8path(tour_dir_) / (shot_name_ + ".ppm")).u8string();
  if (FILE* f = std::fopen(path.c_str(), "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", fb_w, fb_h);
    for (int y = fb_h - 1; y >= 0; --y)
      std::fwrite(&px[static_cast<size_t>(y) * static_cast<size_t>(fb_w) * 3], 1, static_cast<size_t>(fb_w) * 3, f);
    std::fclose(f);
  }
  shot_name_.clear();
}
