#include "app.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>

#include "imgui.h"
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
  if (ImGui::BeginCombo(label, preview)) {
    for (size_t i = 0; i < items.size(); ++i) {
      const bool sel = (i == cur);
      if (ImGui::Selectable(shown ? (*shown)[i].c_str() : items[i].c_str(), sel)) {
        value = items[i];
        changed = true;
      }
      if (sel) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
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

void PushHist(std::vector<float>& h, float v, size_t max) {
  h.push_back(v);
  if (h.size() > max) h.erase(h.begin(), h.begin() + static_cast<std::ptrdiff_t>(h.size() - max));
}

}  // namespace

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
}

App::~App() {
  if (be_.Connected()) {
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
            {"auto_start_backend", true}, {"carla_host", "localhost"}, {"carla_port", 2000},
            {"last_config", ""}, {"dark_theme", true}};
  std::ifstream pf(fs::u8path(prefs_path_));
  if (pf) {
    json saved = json::parse(pf, nullptr, false);
    if (saved.is_object()) prefs_.update(saved);
  }
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](std::string& dst) { if (i + 1 < argc) dst = argv[++i]; };
    std::string v;
    if (a == "--tour") next(tour_dir_);
    else if (a == "--python") { next(v); prefs_["python"] = v; }
    else if (a == "--backend-dir") { next(v); prefs_["backend_dir"] = v; }
    else if (a == "--carla-port") { next(v); prefs_["carla_port"] = std::atoi(v.c_str()); }
    else if (a == "--font") next(font_path_);
    else if (a == "--config") { next(v); prefs_["last_config"] = v; }
    else if (a == "--light") prefs_["dark_theme"] = false;
    else if (a == "--auto-connect") auto_connect_ = true;
  }
  dark_ = prefs_.value("dark_theme", true);

  std::string last = prefs_.value("last_config", std::string());
  if (!last.empty() && plat::FileExists(last)) LoadConfig(last);
  if (prefs_.value("auto_start_backend", true)) StartBackend();
  if (!tour_dir_.empty()) BuildTour();
}

void App::SavePrefs() {
  prefs_["dark_theme"] = dark_;
  std::ofstream f(fs::u8path(prefs_path_));
  if (f) f << prefs_.dump(2);
}

void App::Log(const std::string& msg, const std::string& level) {
  log_.push_back({level, NowStr(), msg});
  while (log_.size() > 800) log_.pop_front();
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
  if (plat::IsAlive(backend_proc_)) return;
  std::string dir = prefs_.value("backend_dir", std::string());
  std::string script = (fs::u8path(dir) / "backend_server.py").u8string();
  if (dir.empty() || !plat::FileExists(script)) {
    Log("找不到 backend_server.py，请在“连接”页设置桥接目录", "error");
    return;
  }
  std::string log_path = (fs::u8path(dir) / "backend.log").u8string();
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
  if (be_.Connected()) {
    be_.Request("shutdown", json::object(), nullptr);
    for (int i = 0; i < 30 && plat::IsAlive(backend_proc_); ++i) glfwWaitEventsTimeout(0.1);
  }
  be_.Disconnect();
  if (backend_proc_.valid()) plat::Kill(backend_proc_);
  carla_connected_ = false;
  Log("后端已停止，CARLA 中的主车、传感器和交通已清理");
}

void App::ConnectBackend() {
  std::string err;
  if (!be_.Connect("127.0.0.1", prefs_.value("backend_port", 57100), err)) {
    Log(err, "error");
    return;
  }
  Log("已连接后端");
  if (!cfg_.contains("carsim")) {
    Call("default_config", json::object(), [this](const json& r) {
      json carla = cfg_["carla"];
      cfg_ = r;
      cfg_["carla"].update(carla);
      if (!cfg_["drive"].contains("cosim_driver")) cfg_["drive"]["cosim_driver"] = cfg_["run"].value("driver", std::string("custom"));
      RefreshDisk();
    });
  }
  Call("rig_presets", json::object(), [this](const json& r) { rig_presets_ = r; });
}

void App::ConnectCarla() {
  if (!be_.Connected()) ConnectBackend();
  json args = {{"host", prefs_.value("carla_host", std::string("localhost"))},
               {"port", prefs_.value("carla_port", 2000)}};
  Call("connect", args, [this](const json& r) {
    carla_connected_ = true;
    server_info_ = r;
    world_ = r;
    map_choice_ = r.value("map", std::string());
    SavePrefs();
    RefreshAfterMapChange();
    Call("list_maps", json::object(), [this](const json& m) { maps_ = m.get<std::vector<std::string>>(); });
    Call("list_weathers", json::object(), [this](const json& w) { weathers_ = w.get<std::vector<std::string>>(); });
    RefreshVehicles();
  }, "正在连接 CARLA ...");
}

void App::RefreshWorld() {
  Call("world_info", json::object(), [this](const json& r) {
    world_ = r;
    weather_edit_ = r.value("weather", json::object());
  });
}

void App::RefreshAfterMapChange() {
  RefreshWorld();
  RefreshSpawnPoints();
  RefreshActors();
  view_on_ = false;
}

void App::RefreshDisk() {
  std::string dir = cfg_.contains("collect") ? cfg_["collect"].value("out_dir", std::string("datasets"))
                                             : std::string(".");
  Call("disk_info", {{"path", dir}}, [this](const json& r) { disk_ = r; });
}

void App::LoadMap(const std::string& name) {
  Call("load_map", {{"name", name}}, [this](const json& r) {
    world_ = r;
    map_choice_ = r.value("map", std::string());
    Log("地图已切换为 " + r.value("map", std::string()));
    RefreshAfterMapChange();
  }, "正在加载地图 " + name + " ...");
}

void App::ApplyWeatherPreset(const std::string& preset) {
  Call("set_weather", {{"preset", preset}}, [this, preset](const json& r) {
    weather_edit_ = r;
    Log("天气：" + preset);
  });
}

void App::ApplyWeatherParams() {
  Call("set_weather", {{"params", weather_edit_}}, [this](const json& r) {
    weather_edit_ = r;
    Log("天气参数已应用");
  });
}

void App::ApplyWorldSettings() {
  json a = {{"synchronous", world_.value("synchronous", false)},
            {"frame_dt", world_.value("frame_dt", 0.0)},
            {"no_rendering", world_.value("no_rendering", false)},
            {"idle_tick", world_.value("idle_tick", false)}};
  Call("world_settings", a, [this](const json& r) {
    world_ = r;
    Log("仿真设置已应用");
  });
}

void App::RefreshVehicles() {
  Call("list_vehicles", json::object(), [this](const json& r) {
    vehicles_ = r;
    const std::string want = cfg_["carla"].value("vehicle", std::string());
    for (size_t i = 0; i < vehicles_.size(); ++i)
      if (vehicles_[i]["id"] == want) vehicle_sel_ = static_cast<int>(i);
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
  // The GUI keeps the CarSim driver in drive.cosim_driver; the backend reads run.driver.
  if (cfg_.contains("drive") && cfg_["drive"].contains("cosim_driver"))
    cfg_["run"]["driver"] = cfg_["drive"]["cosim_driver"];
  for (auto* h : {&h_t_, &h_speed_, &h_steer_fl_, &h_steer_fr_, &h_rt_, &h_thr_, &h_brk_}) h->clear();
  for (auto& h : h_susp_) h.clear();
  trail_x_.clear();
  trail_y_.clear();
  last_tel_ = json::object();
  collect_stats_ = json::object();
  Call("cosim_start", {{"config", cfg_}}, [this](const json& r) {
    run_info_ = r;
    RefreshWorld();
  }, "正在启动 ...");
}

void App::RunCommand(const std::string& cmd) { Call(cmd, json::object(), nullptr); }

void App::StartView(const json& mount, float fov) {
  static const int kRes[][2] = {{480, 270}, {640, 360}, {960, 540}, {1280, 720}};
  json a = {{"mode", view_mode_}, {"width", kRes[view_res_][0]}, {"height", kRes[view_res_][1]}, {"fps", 15}};
  if (!mount.is_null()) {
    a["mount"] = mount;
    a["fov"] = fov;
  } else {
    view_rig_sensor_.clear();
  }
  Call("view_start", a, [this](const json&) { view_on_ = true; });
}

void App::UploadViewTexture() {
  if (!view_dirty_) return;
  view_dirty_ = false;
  if (!view_tex_) {
    GLuint t = 0;
    glGenTextures(1, &t);
    view_tex_ = t;
    glBindTexture(GL_TEXTURE_2D, view_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F);  // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F);
  }
  glBindTexture(GL_TEXTURE_2D, view_tex_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, view_w_, view_h_, 0, GL_RGB, GL_UNSIGNED_BYTE, view_pixels_.data());
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

void App::LoadConfig(const std::string& path) {
  std::ifstream f(fs::u8path(path));
  json j = f ? json::parse(f, nullptr, false) : json();
  if (!j.is_object()) {
    Log("无法读取配置：" + path, "error");
    return;
  }
  cfg_.merge_patch(j);
  cfg_path_ = path;
  prefs_["last_config"] = path;
  SavePrefs();
  Log("已载入配置 " + path);
}

void App::SaveConfig(const std::string& path) {
  std::ofstream f(fs::u8path(path));
  if (!f) {
    Log("无法写入 " + path, "error");
    return;
  }
  f << cfg_.dump(2);
  cfg_path_ = path;
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
  } else if (type == "cosim_state") {
    run_state_ = ev.value("state", std::string());
    if (!Running()) {
      RefreshWorld();
      RefreshDisk();
      kb_throttle_ = kb_brake_ = kb_steer_ = 0;
    }
  } else if (type == "telemetry") {
    last_tel_ = ev["data"];
    const json& d = last_tel_;
    PushHist(h_t_, d.value("t", 0.0f), kHist);
    if (d.contains("location") && d["location"].size() >= 2) {
      PushHist(trail_x_, d["location"][0].get<float>(), 6000);
      PushHist(trail_y_, d["location"][1].get<float>(), 6000);
    }
    PushHist(h_speed_, d.value("speed_kmh", 0.0f), kHist);
    PushHist(h_rt_, d.value("rt_factor", 0.0f), kHist);
    const json& st = d["wheel_steer"];
    PushHist(h_steer_fl_, st.size() > 0 ? st[0].get<float>() : 0.0f, kHist);
    PushHist(h_steer_fr_, st.size() > 1 ? st[1].get<float>() : 0.0f, kHist);
    const json& su = d["wheel_suspension_mm"];
    for (size_t i = 0; i < 4; ++i) PushHist(h_susp_[i], su.size() > i ? su[i].get<float>() : 0.0f, kHist);
    const json& a = d["action"];
    PushHist(h_thr_, a.size() > 0 ? a[0].get<float>() : 0.0f, kHist);
    PushHist(h_brk_, a.size() > 1 ? a[1].get<float>() : 0.0f, kHist);
  } else if (type == "collect_stats") {
    collect_stats_ = ev;
  } else if (type == "frame") {
    if (view_on_ && Base64Decode(ev.value("rgb", std::string()), view_pixels_)) {
      view_w_ = ev.value("w", 0);
      view_h_ = ev.value("h", 0);
      if (static_cast<int>(view_pixels_.size()) >= view_w_ * view_h_ * 3) {
        view_dirty_ = true;
        ++view_frames_;
      }
    }
  } else if (type == "progress") {
    const int done = ev.value("done", 0), total = ev.value("total", 1);
    if (done < total) busy_ = Fmt("正在测量车型尺寸 %d/%d", done + 1, total);
  } else if (type == "disconnected") {
    carla_connected_ = false;
    Log("与后端的连接断开了", "error");
  }
}

// --------------------------------------------------------------------------
// --tour: scripted walk through every panel, saving screenshots
// --------------------------------------------------------------------------
void App::BuildTour() {
  fs::create_directories(fs::u8path(tour_dir_));
  auto idle = [this] { return busy_.empty() && be_.PendingCount() == 0; };
  const std::string ds = (fs::u8path(tour_dir_) / "tour_dataset").u8string();
  tour_ = new std::vector<TourStep>{
      {kPanelConnect, [this] { ConnectBackend(); }, [this] { return be_.Connected(); }, ""},
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
      {kPanelTraffic, [this] { traffic_vehicles_ = 12; traffic_walkers_ = 8; SpawnTraffic(); }, idle, "06_traffic"},
      {kPanelDrive, [this] {
         cfg_["drive"]["dynamics"] = "cosim";
         cfg_["drive"]["cosim_driver"] = "custom";
         cfg_["drive"]["target_speed_kmh"] = 35.0;
       }, idle, "07_drive"},
      {kPanelCoSim, [this] {
         cfg_["carsim"]["mock"] = true;
         cfg_["sync"]["duration"] = 10.0;
         cfg_["run"]["log_path"] = "";
       }, idle, "08_cosim_config"},
      {kPanelCollect, [this, ds] {
         cfg_["collect"]["enabled"] = false;
         cfg_["collect"]["out_dir"] = ds;
       }, idle, "09_collect_config"},
      {kPanelView, [this] { view_mode_ = "chase"; view_res_ = 1; StartView(); }, idle, ""},
      {kPanelView, [this] { StartRun(); },
       [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 5.0; }, "10_running_view"},
      {kPanelDrive, [] {}, [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 7.0; }, "11_running_drive"},
      {kPanelDrive, [] {}, [this] { return !Running(); }, ""},
      // Tiny capture (3 frames of a single small camera) to show live stats;
      // the tour output folder is deleted by the caller afterwards.
      {kPanelCollect, [this] {
         cfg_["drive"]["dynamics"] = "carla";
         cfg_["drive"]["carla_driver"] = "autopilot";
         cfg_["drive"]["tm_ignore_lights"] = true;
         cfg_["sync"]["frame_dt"] = 0.1;
         cfg_["sync"]["duration"] = 0.0;
         cfg_["rig"]["sensors"] = json::array();
         cfg_["rig"]["preset"] = "front_camera";
         cfg_["collect"]["enabled"] = true;
         cfg_["collect"]["max_frames"] = 3;
         cfg_["collect"]["max_gb"] = 0.1;
         StartRun();
       }, [this] { return !Running() && collect_stats_.value("frames", 0) >= 3; }, "12_collect_done"},
      {kPanelActors, [this] { ClearTraffic(); RefreshActors(); }, idle, "13_actors"},
      {kPanelRecorder, [] {}, idle, "14_recorder"},
      {kPanelWorld, [this] { LoadMap("Town03"); }, [this, idle] { return idle() && world_.value("map", "") == "Town03"; },
       "15_map_town03"},
      {kPanelWorld, [this] { dark_ = false; theme_changed_ = true; }, idle, "16_light_theme"},
      {kPanelWorld, [this] { dark_ = true; theme_changed_ = true; LoadMap("Town10HD_Opt"); },
       [this, idle] { return idle() && world_.value("map", "") == "Town10HD_Opt"; }, ""},
  };
}

void App::TourTick() {
  static bool started = false;
  static int since_ready = 0, frames_in_step = 0;
  if (tour_i_ >= tour_->size()) {
    if (++tour_wait_ > 30) {
      Log("TOUR DONE");
      quit_ = true;
    }
    return;
  }
  TourStep& s = (*tour_)[tour_i_];
  panel_ = s.panel;
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
    Log("TOUR step " + std::to_string(tour_i_) + " TIMEOUT", "error");
    if (!s.shot.empty()) shot_name_ = s.shot + "_TIMEOUT";
    ++tour_i_;
    started = false;
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
