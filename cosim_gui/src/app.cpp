#include "app.h"

#include <algorithm>
#include <cfloat>
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

bool appui::Base64(const std::string& in, std::vector<unsigned char>& out) { return Base64Decode(in, out); }

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
    else if (a == "--hero") { next(tour_dir_); hero_spawns_ = hero_spawns_.empty() ? "3" : hero_spawns_; }
    else if (a == "--hero-spawns") next(hero_spawns_);
    else if (a == "--python") { next(v); prefs_["python"] = v; }
    else if (a == "--backend-dir") { next(v); prefs_["backend_dir"] = v; }
    else if (a == "--carla-port") { next(v); prefs_["carla_port"] = std::atoi(v.c_str()); }
    else if (a == "--font") next(font_path_);
    else if (a == "--config") { next(v); prefs_["last_config"] = v; }
    else if (a == "--light") prefs_["dark_theme"] = false;
    else if (a == "--auto-connect") auto_connect_ = true;
    else if (a == "--size" || a == "--scale") next(v);  // handled in main.cpp
  }
  dark_ = prefs_.value("dark_theme", true);

  std::string last = prefs_.value("last_config", std::string());
  if (!last.empty() && plat::FileExists(last)) LoadConfig(last);
  if (prefs_.value("auto_start_backend", true)) StartBackend();
  if (!tour_dir_.empty()) {
    if (hero_spawns_.empty()) BuildTour(); else BuildHeroTour();
  }
}

void App::SavePrefs() {
  prefs_["dark_theme"] = dark_;
  std::ofstream f(fs::u8path(prefs_path_));
  if (f) f << prefs_.dump(2, ' ', false, json::error_handler_t::replace);
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
  // Always start from the backend's full defaults and lay what we already have
  // (e.g. a loaded, possibly partial config file) on top, so every section the
  // pages read exists.
  Call("default_config", json::object(), [this](const json& r) {
    json cur = cfg_.is_object() ? cfg_ : json::object();
    cfg_ = r;
    cfg_.merge_patch(cur);
    if (!cfg_["drive"].contains("cosim_driver")) cfg_["drive"]["cosim_driver"] = cfg_["run"].value("driver", std::string("custom"));
    RefreshDisk();
  });
  Call("rig_presets", json::object(), [this](const json& r) { rig_presets_ = r; });
}

void App::ConnectCarla() {
  if (!be_.Connected()) ConnectBackend();
  json args = {{"host", prefs_.value("carla_host", std::string("localhost"))},
               {"port", prefs_.value("carla_port", 2000)}};
  Call("connect", args, [this](const json& r) {
    carla_connected_ = true;
    server_info_ = r;
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
  if (!Running() && r.is_object() && r.value("ego_id", 0) != world_.value("ego_id", 0)) run_note_.clear();
  world_ = r.is_object() ? r : json::object();
}

void App::RefreshWorld() {
  Call("world_info", json::object(), [this](const json& r) {
    SetWorld(r);
    // The backend's run state is authoritative (e.g. after a reconnect).
    if (r.contains("cosim_state") && r["cosim_state"].is_string()) run_state_ = r["cosim_state"].get<std::string>();
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
    SetWorld(r);
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
    SetWorld(r);
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
  if (!mount.is_null()) {
    view_rig_mount_ = mount;
    view_rig_fov_ = fov;
  } else {
    view_rig_sensor_.clear();
    view_rig_mount_ = json();
  }
  SendViews();
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
      v["mount"] = json{{"x", s.value("x", 0.0)}, {"y", s.value("y", 0.0)}, {"z", s.value("z", 0.0)}, {"pitch", s.value("pitch", 0.0)}, {"yaw", s.value("yaw", 0.0)}, {"roll", s.value("roll", 0.0)}};
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
  std::vector<std::pair<std::string, std::string>> out = {
      {"cam:chase", "相机 · 跟车"}, {"cam:hood", "相机 · 车头"}, {"cam:wheel", "相机 · 前轮特写"}, {"cam:top", "相机 · 俯视"},
      {"semantic", "语义分割（前视）"}, {"depth", "深度（前视）"}, {"instance", "实例分割（前视）"},
      {"lidar", "激光雷达点云（俯视）"}, {"radar", "毫米波雷达（俯视）"}};
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
  f << cfg_.dump(2, ' ', false, json::error_handler_t::replace);
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
    // Say clearly why a run ended on its own; a parked car otherwise looks stuck.
    if (run_state_ == "finished" || run_state_ == "error") {
      const double dur = cfg_.contains("sync") ? cfg_["sync"].value("duration", 0.0) : 0.0;
      const double t = last_tel_.value("t", 0.0);
      if (run_state_ == "error") {
        run_note_level_ = "error";
        run_note_ = "运行出错已停止：" + ev.value("detail", std::string()) + "（详情见底部“输出”）";
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
      Log(run_note_, run_note_level_);
    } else {
      run_note_.clear();
    }
    if (!Running()) {
      // The car is parked now: clear the live readouts so the viewport HUD
      // does not keep showing the last speed (the plots keep the history).
      last_tel_ = json::object();
      RefreshWorld();
      RefreshDisk();
      kb_throttle_ = kb_brake_ = kb_steer_ = 0;
    }
  } else if (type == "telemetry") {
    last_tel_ = ev.value("data", json::object());
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
    const json& st = d["wheel_steer"];
    PushHist(h_steer_fl_, st.size() > 0 ? static_cast<float>(appui::NumAt(st, 0)) : 0.0f, kHist);
    PushHist(h_steer_fr_, st.size() > 1 ? static_cast<float>(appui::NumAt(st, 1)) : 0.0f, kHist);
    const json& su = d["wheel_suspension_mm"];
    for (size_t i = 0; i < 4; ++i) PushHist(h_susp_[i], su.size() > i ? static_cast<float>(appui::NumAt(su, i)) : 0.0f, kHist);
    const json& a = d["action"];
    PushHist(h_thr_, a.size() > 0 ? static_cast<float>(appui::NumAt(a, 0)) : 0.0f, kHist);
    PushHist(h_brk_, a.size() > 1 ? static_cast<float>(appui::NumAt(a, 1)) : 0.0f, kHist);
  } else if (type == "collect_stats") {
    collect_stats_ = ev;
  } else if (type == "frame") {
    const std::string vid = ev.value("view", std::string("p0"));
    if (vid.size() == 2 && vid[1] >= '1' && vid[1] <= '3') {
      ViewPane& pane = panes_[vid[1] - '0'];
      if (view_on_ && Base64Decode(ev.value("rgb", std::string()), pane.px)) {
        pane.w = ev.value("w", 0);
        pane.h = ev.value("h", 0);
        if (static_cast<int>(pane.px.size()) >= pane.w * pane.h * 3) {
          pane.dirty = true;
          ++pane.frames;
        }
      }
    } else if (view_on_ && Base64Decode(ev.value("rgb", std::string()), view_pixels_)) {
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
    if (ev.value("ok", false))
      Log(Fmt("导出完成：%s", ev["result"].value("out", std::string()).c_str()));
    else
      Log("导出失败：" + ev.value("error", std::string()), "error");
  } else if (type == "disconnected") {
    // Nothing the backend owned is valid any more: never stay "running" or busy.
    carla_connected_ = false;
    if (Running()) run_state_ = "error";
    busy_.clear();
    view_on_ = false;
    ds_play_ = false;
    ds_pending_ = 0;
    ds_dirty_ = false;
    ds_exporting_ = false;
    last_tel_ = json::object();
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
      {kPanelTraffic, [this] { traffic_vehicles_ = 12; traffic_walkers_ = 8; SpawnTraffic(); }, idle, "06_traffic"},
      {kPanelDrive, [this] {
         cfg_["drive"]["dynamics"] = "cosim";
         cfg_["drive"]["cosim_driver"] = "custom";
         cfg_["drive"]["target_speed_kmh"] = 35.0;
       }, idle, "07_drive"},
      {kPanelCoSim, [this] {
         cfg_["carsim"]["mock"] = true;
         cfg_["sync"]["duration"] = 0.0;  // stopped below with the toolbar button
         cfg_["run"]["log_path"] = "";
       }, idle, "08_cosim_config"},
      {kPanelCollect, [this, ds] {
         cfg_["collect"]["enabled"] = false;
         cfg_["collect"]["out_dir"] = ds;
       }, idle, "09_collect_config"},
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
      {kPanelDrive, [] {}, [this] { return run_state_ == "running" && last_tel_.value("t", 0.0) > 7.0; }, "11_running_drive"},
      {kPanelDrive, [this] { click_target_ = "暂停"; }, [this] { return run_state_ == "paused"; }, ""},
      {kPanelDrive, [this] { tour_mark_ = last_tel_.value("frame", 0); click_target_ = "单步"; },
       [this] { return run_state_ == "paused" && last_tel_.value("frame", 0) == tour_mark_ + 1; }, ""},
      {kPanelDrive, [this] { click_target_ = "继续"; }, [this] { return run_state_ == "running"; }, ""},
      {kPanelDrive, [this] { click_target_ = "停止"; },
       [this] { return run_state_ == "stopped" && last_tel_.empty(); }, "11b_stopped"},
      {kPanelDrive, [this] { click_target_ = "view:wheel"; },
       [this] { return view_on_ && view_mode_ == "wheel" && busy_.empty() && views_pending_ == 0 && view_frames_ > 3; }, ""},
      {kPanelDrive, [this] { click_target_ = "view:close"; }, [this] { return !view_on_; }, ""},
      {kPanelDrive, [this] { click_target_ = "viewport:action"; }, [this] { return view_on_; }, ""},
      {-1, [this] { click_target_ = "tab:传感器"; }, [this] { return panel_ == kPanelRig; }, ""},
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
             {{"name", "cam_front"}, {"type", "rgb"}, {"x", 1.5}, {"y", 0.0}, {"z", 1.6}, {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
              {"attributes", {{"image_size_x", 640}, {"image_size_y", 360}, {"fov", 90.0}}}, {"enabled", true}},
             {{"name", "lidar_top"}, {"type", "lidar"}, {"x", 0.0}, {"y", 0.0}, {"z", 1.9}, {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
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
      {kPanelDataset, [this] { cfg_["collect"]["enabled"] = false; click_target_ = "ds:refresh"; },
       [this, idle] { return idle() && ds_sessions_.is_array() && !ds_sessions_.empty(); }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:session:tour"; },
       [this, idle] { return idle() && !ds_root_.empty() && ds_pending_ == 0 && ds_panes_[0].frames > 0 && ds_panes_[1].frames > 0; }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:next"; }, [this, idle] { return idle() && ds_idx_ == 1 && ds_pending_ == 0; }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:play"; },
       [this, idle] { return idle() && !ds_play_ && ds_idx_ == DatasetFrameCount() - 1 && ds_pending_ == 0; }, "12b_dataset_browser"},
      {kPanelDataset, [this] { ds_fmt_ = 0; props_scroll_end_ = true; }, [] { return true; }, ""},
      {kPanelDataset, [this] { click_target_ = "ds:export"; },
       [this, idle] { return idle() && !ds_exporting_ && ds_export_result_.value("ok", false); }, "12c_dataset_export"},
      {kPanelActors, [this] { ClearTraffic(); RefreshActors(); }, idle, "13_actors"},
      {kPanelRecorder, [] {}, idle, "14_recorder"},
      {kPanelWorld, [this] { LoadMap("Town03"); }, [this, idle] { return idle() && world_.value("map", "") == "Town03" && run_note_.empty(); },
       "15_map_town03"},  // the "ego parked" banner of the last run is gone with the ego
      {kPanelWorld, [this] { dark_ = false; theme_changed_ = true; }, idle, "16_light_theme"},
      {kPanelWorld, [this] { dark_ = true; theme_changed_ = true; LoadMap("Town10HD_Opt"); },
       [this, idle] { return idle() && world_.value("map", "") == "Town10HD_Opt"; }, ""},
  };
}

// A CarSim co-simulation (mock CarSim, route following on the road) with
// traffic and the 1+3 multi-view, run from several spawn points; screenshots
// at a few moments of each run to pick a README cover from.
void App::BuildHeroTour() {
  fs::create_directories(fs::u8path(tour_dir_));
  auto idle = [this] { return busy_.empty() && be_.PendingCount() == 0; };
  tour_ = new std::vector<TourStep>{
      {kPanelConnect, [this] { ConnectBackend(); }, [this] { return be_.Connected(); }, ""},
      {kPanelConnect, [this] { ConnectCarla(); }, [this, idle] { return carla_connected_ && idle(); }, ""},
      {kPanelWorld, [this] { ApplyWeatherPreset("ClearNoon"); }, idle, ""},
      {kPanelVehicle, [this] {
         for (size_t i = 0; i < vehicles_.size(); ++i)
           if (vehicles_[i]["id"] == "vehicle.tesla.model3") vehicle_sel_ = static_cast<int>(i);
         SpawnEgo();
       }, [this, idle] { return idle() && world_.value("ego_id", 0) != 0; }, ""},
      {kPanelTraffic, [this] { traffic_vehicles_ = 40; traffic_walkers_ = 20; SpawnTraffic(); }, idle, ""},
      {kPanelView, [this] {
         cfg_["drive"]["dynamics"] = "cosim";
         cfg_["drive"]["cosim_driver"] = "route";
         cfg_["drive"]["target_speed_kmh"] = 35.0;
         cfg_["carsim"]["mock"] = true;
         cfg_["sync"]["duration"] = 0.0;
         cfg_["sync"]["frame_dt"] = 0.05;
         cfg_["run"]["log_path"] = "";
         cfg_["collect"]["enabled"] = false;
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
    tour_->push_back({kPanelView, [this, sp] { cfg_["carla"]["spawn_index"] = sp; StartRun(); },
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
    if (++tour_wait_ > 30) {
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
  if (click_target_.empty()) return;
  ImVec2 c;
  if (!ui::FindTarget(click_target_, &c)) {
    if (++click_phase_ > 120) {
      Log("TOUR click target missing: " + click_target_, "error");
      click_target_.clear();
      click_phase_ = 0;
    }
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
