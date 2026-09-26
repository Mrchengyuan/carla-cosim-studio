// All pages except the sensor-rig editor.
#include <algorithm>
#include <cmath>

#include "app.h"
#include "imgui.h"
#include "implot.h"
#include "ui_kit.h"

using appui::ComboStr;
using appui::Fmt;
using appui::InputStr;

namespace {

bool EditDouble(json& obj, const char* key, double step, const char* fmt, double lo = -1e18, double hi = 1e18) {
  double v = obj.value(key, 0.0);
  if (ImGui::InputDouble(Fmt("##%s", key).c_str(), &v, step, step * 10, fmt)) {
    obj[key] = std::max(lo, std::min(hi, v));
    return true;
  }
  return false;
}

bool EditString(json& obj, const char* key) {
  std::string v = obj.value(key, std::string());
  if (InputStr(Fmt("##%s", key).c_str(), v)) {
    obj[key] = v;
    return true;
  }
  return false;
}

// Radio option row used for mode choices (dynamics / driver): radio mark,
// icon, bold title and a one-line description.
bool ChoiceCard(const char* id, const char* icon, const char* title, const char* desc, bool selected, float w) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushID(id);
  ImVec2 pos = ImGui::GetCursorScreenPos();
  // The description wraps (narrow panels, small screens) and the card grows.
  const float desc_size = fs * 0.88f, wrap = std::max(fs * 4.0f, w - fs * 4.1f);
  const float desc_h = ImGui::GetFont()->CalcTextSizeA(desc_size, FLT_MAX, wrap, desc).y;
  const float h = std::max(fs * 2.75f, fs * 1.65f + desc_h);
  const bool clicked = ImGui::InvisibleButton("##c", ImVec2(w, h));
  const bool hov = ImGui::IsItemHovered();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec4 bg = selected ? ui::WithAlpha(p.accent, ui::IsDark() ? 0.14f : 0.08f)
                  : hov ? ui::WithAlpha(p.text, 0.04f) : p.field;
  dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + h), ImGui::GetColorU32(bg), 2.0f);
  dl->AddRect(pos, ImVec2(pos.x + w, pos.y + h), ImGui::GetColorU32(selected ? p.accent : p.card_border), 2.0f);
  const ImVec2 rc(pos.x + fs * 1.0f, pos.y + h * 0.5f);
  dl->AddCircle(rc, fs * 0.42f, ImGui::GetColorU32(selected ? p.accent : p.text_dim), 24, 1.3f);
  if (selected) dl->AddCircleFilled(rc, fs * 0.22f, ImGui::GetColorU32(p.accent), 24);
  dl->AddText(ImVec2(pos.x + fs * 2.0f, pos.y + fs * 0.42f), ImGui::GetColorU32(selected ? p.accent : p.text_dim), icon);
  ImFont* bold = ui::GetFonts().bold ? ui::GetFonts().bold : ImGui::GetFont();
  dl->AddText(bold, fs, ImVec2(pos.x + fs * 3.5f, pos.y + fs * 0.42f), ImGui::GetColorU32(p.text), title);
  dl->AddText(ImGui::GetFont(), desc_size, ImVec2(pos.x + fs * 3.5f, pos.y + fs * 1.5f), ImGui::GetColorU32(p.text_dim),
              desc, nullptr, wrap);
  ImGui::PopID();
  return clicked;
}

void KeyCap(const char* k, bool down) {
  const ui::Palette& p = ui::Colors();
  const float s = ImGui::GetFontSize() * 2.0f;
  ImVec2 pos = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec4 bg = down ? p.accent : p.card;
  dl->AddRectFilled(pos, ImVec2(pos.x + s, pos.y + s), ImGui::GetColorU32(bg), 5.0f);
  dl->AddRect(pos, ImVec2(pos.x + s, pos.y + s), ImGui::GetColorU32(p.card_border), 5.0f);
  ImVec2 ts = ImGui::CalcTextSize(k);
  dl->AddText(ImVec2(pos.x + (s - ts.x) * 0.5f, pos.y + (s - ts.y) * 0.5f),
              ImGui::GetColorU32(down ? ImVec4(1, 1, 1, 1) : p.text), k);
  ImGui::Dummy(ImVec2(s, s));
}

}  // namespace

// --------------------------------------------------------------------------
void App::DrawViewImage(float max_w, float max_h) {
  if (!view_tex_ || view_w_ <= 0 || view_h_ <= 0) {
    ImGui::TextDisabled("等待画面 ...");
    return;
  }
  const float w = static_cast<float>(view_w_), h = static_cast<float>(view_h_);
  const float s = std::min(max_w / w, max_h / h);
  ImGui::Image(static_cast<ImTextureID>(static_cast<intptr_t>(view_tex_)), ImVec2(w * s, h * s));
}

// --------------------------------------------------------------------------
void App::DrawPanelConnect() {
  const ui::Palette& p = ui::Colors();
  ui::BeginCard(ICON_FA_SERVER, "CARLA 服务器");
  std::string host = prefs_.value("carla_host", std::string("localhost"));
  ui::Row("主机地址");
  if (InputStr("##host", host)) prefs_["carla_host"] = host;
  int cport = prefs_.value("carla_port", 2000);
  ui::Row("RPC 端口", "原版 CARLA 默认 2000；服务器上的改版 CARLA 用的是 3000", ImGui::GetFontSize() * 8);
  if (ImGui::InputInt("##cport", &cport)) prefs_["carla_port"] = cport;
  ImGui::Dummy(ImVec2(0, 2));
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  ImGui::BeginDisabled(!be_.Connected() || !busy_.empty() || Running());  // reconnecting ends the run's world
  if (ui::Button(ICON_FA_PLUG, carla_connected_ ? "重新连接" : "连接 CARLA", ui::Kind::Primary)) ConnectCarla();
  ImGui::EndDisabled();
  if (carla_connected_) {
    ImGui::Separator();
    ui::Row("服务器版本");
    ImGui::TextUnformatted(server_info_.value("server_version", std::string()).c_str());
    ui::Row("客户端版本");
    ImGui::TextUnformatted(server_info_.value("client_version", std::string()).c_str());
    ui::Row("当前地图");
    ImGui::TextUnformatted(world_.value("map", std::string()).c_str());
    ui::Row("外部动力学接口", "改版 CARLA 一帧一次下发完整状态，速度 / IMU 读数真实，悬架同步可用");
    // The client may be the patched one while the server is the original:
    // the server side is only known after the first co-sim run.
    const json srv = world_.contains("external_api_server") ? world_["external_api_server"] : json();
    if (!server_info_.value("external_api_client", false))
      ui::Pill(ICON_FA_TRIANGLE_EXCLAMATION " 原版 CARLA：兼容模式", p.warning);
    else if (srv.is_boolean() && !srv.get<bool>())
      ui::Pill(ICON_FA_TRIANGLE_EXCLAMATION " 服务器是原版 CARLA：兼容模式", p.warning);
    else if (srv.is_boolean())
      ui::Pill(ICON_FA_CIRCLE_CHECK " 改版 CARLA：可用", p.success);
    else
      ui::Pill(ICON_FA_CIRCLE_INFO " 改版客户端：服务器是否改版在第一次运行时确认", p.accent);
  }
  ui::EndCard();

  ui::BeginCard(ICON_FA_TERMINAL, "Python 后端");
  std::string py = prefs_.value("python", std::string());
  ui::Row("Python 解释器", "装有 carla 包的 Python（Windows 上一般是 python 或虚拟环境里的 python.exe）");
  if (InputStr("##py", py)) prefs_["python"] = py;
  std::string dir = prefs_.value("backend_dir", std::string());
  ui::Row("桥接目录", "carsim_carla_bridge 目录，里面有 backend_server.py");
  if (InputStr("##dir", dir)) prefs_["backend_dir"] = dir;
  int port = prefs_.value("backend_port", 57100);
  ui::Row("后端端口", nullptr, ImGui::GetFontSize() * 8);
  if (ImGui::InputInt("##bport", &port)) prefs_["backend_port"] = port;
  bool autostart = prefs_.value("auto_start_backend", true);
  ui::Row("自动启动");
  if (ImGui::Checkbox("##auto", &autostart)) prefs_["auto_start_backend"] = autostart;
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  const bool alive = plat::IsAlive(backend_proc_);
  ImGui::BeginDisabled(alive);
  if (ui::Button(ICON_FA_PLAY, "启动后端")) StartBackend();
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!alive && !be_.Connected());
  if (ui::Button(ICON_FA_STOP, "停止后端")) StopBackend();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ui::Button(ICON_FA_FLOPPY_DISK, "保存设置")) SavePrefs();
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelWorld() {
  const float fs = ImGui::GetFontSize();
  ui::BeginCard(ICON_FA_MAP, "地图");
  ui::Row("地图", "切换地图会清除主车、交通和传感器；带 _Opt 的是分层地图");
  ComboStr("##map", map_choice_, maps_);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  ImGui::BeginDisabled(!busy_.empty() || map_choice_.empty() || Running());
  if (ui::Button(ICON_FA_FOLDER_OPEN, "加载地图", ui::Kind::Primary)) LoadMap(map_choice_);
  ImGui::SameLine();
  if (ui::Button(ICON_FA_ROTATE, "重载当前地图"))
    Call("reload_world", json::object(), [this](const json& r) { SetWorld(r); RefreshAfterMapChange(); }, "正在重载地图 ...");
  ImGui::EndDisabled();
  ui::EndCard();

  ui::BeginCard(ICON_FA_CLOUD_SUN, "天气与光照");
  struct Q { const char* preset; const char* name; };
  static const Q kQuick[] = {{"ClearNoon", "晴天正午"}, {"CloudyNoon", "多云"}, {"WetNoon", "湿滑路面"},
                             {"HardRainNoon", "大雨"}, {"ClearSunset", "日落"}, {"ClearNight", "夜晚"},
                             {"SoftRainNight", "雨夜"}, {"MidRainSunset", "黄昏中雨"}};
  ui::Row("快速预设");
  ImGui::BeginGroup();
  const float qw = std::min(fs * 5.5f, (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 3) / 4);
  for (int i = 0; i < 8; ++i) {
    if (i % 4) ImGui::SameLine();
    if (ImGui::Button(kQuick[i].name, ImVec2(qw, 0))) ApplyWeatherPreset(kQuick[i].preset);
  }
  ImGui::EndGroup();
  ui::Row("全部预设");
  ImGui::SetNextItemWidth(std::min(fs * 26, ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize("应用").x -
                                                ImGui::GetStyle().FramePadding.x * 2 - ImGui::GetStyle().ItemSpacing.x));
  ComboStr("##weather", weather_choice_, weathers_);
  ImGui::SameLine();
  if (ImGui::Button("应用")) ApplyWeatherPreset(weather_choice_);
  if (weather_edit_.empty()) weather_edit_ = world_.value("weather", json::object());
  struct W { const char* key; const char* name; float lo, hi; };
  static const W kW[] = {{"sun_altitude_angle", "太阳高度角 °", -90, 90}, {"sun_azimuth_angle", "太阳方位角 °", 0, 360},
                         {"cloudiness", "云量 %", 0, 100}, {"precipitation", "降雨 %", 0, 100},
                         {"precipitation_deposits", "积水 %", 0, 100}, {"wetness", "路面湿度 %", 0, 100},
                         {"wind_intensity", "风力 %", 0, 100}, {"fog_density", "雾浓度 %", 0, 100},
                         {"fog_distance", "雾起始距离 m", 0, 200}};
  for (const auto& w : kW) {
    float v = weather_edit_.value(w.key, 0.0f);
    ui::Row(w.name);
    if (ImGui::SliderFloat(Fmt("##%s", w.key).c_str(), &v, w.lo, w.hi, "%.1f")) weather_edit_[w.key] = v;
  }
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  if (ui::Button(ICON_FA_WAND_MAGIC_SPARKLES, "应用天气参数", ui::Kind::Primary)) ApplyWeatherParams();
  ui::EndCard();

  ui::BeginCard(ICON_FA_SLIDERS, "仿真设置");
  bool sync = world_.value("synchronous", false);
  ui::Row("同步模式", "同步模式下服务器每次 tick 才前进一帧；运行仿真时会自动切换为同步，结束后恢复");
  if (ImGui::Checkbox("##sync", &sync)) world_["synchronous"] = sync;
  double dt = world_.value("frame_dt", 0.0);
  ui::Row("固定步长 s", "0 表示可变步长", fs * 8);
  if (ImGui::InputDouble("##dt", &dt, 0.005, 0.01, "%.3f")) world_["frame_dt"] = std::max(0.0, dt);
  bool norender = world_.value("no_rendering", false);
  ui::Row("关闭渲染", "只算物理、不渲染画面，适合只要车辆动力学或加速训练；相机传感器此时没有图像");
  if (ImGui::Checkbox("##norender", &norender)) world_["no_rendering"] = norender;
  bool idle = world_.value("idle_tick", false);
  ui::Row("空闲时持续推进", "同步模式且没有仿真在运行时，由后端按固定步长持续 tick，否则服务器画面会停住");
  if (ImGui::Checkbox("##idle", &idle)) world_["idle_tick"] = idle;
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  if (ui::Button(ICON_FA_CIRCLE_CHECK, "应用", ui::Kind::Primary)) ApplyWorldSettings();
  ImGui::SameLine();
  if (ui::Button(ICON_FA_ROTATE, "刷新")) RefreshWorld();
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelTraffic() {
  const ui::Palette& p = ui::Colors();
  ui::BeginCard(ICON_FA_TRAFFIC_LIGHT, "背景交通流");
  ui::DimWrapped("由 CARLA 交通管理器控制：遵守信号灯、会让行，行人在人行道上随机走动。");
  ui::Row("车辆数");
  ImGui::SliderInt("##tv", &traffic_vehicles_, 0, 150);
  ui::Row("行人数");
  ImGui::SliderInt("##tw", &traffic_walkers_, 0, 150);
  ui::Row("随机种子", "同一种子 + 同一地图 = 同样的交通布置，便于复现实验", ImGui::GetFontSize() * 8);
  ImGui::InputInt("##seed", &traffic_seed_);
  ui::Row("只生成小汽车", "不生成摩托车、自行车和卡车");
  ImGui::Checkbox("##safe", &traffic_safe_);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  ImGui::BeginDisabled(!busy_.empty());
  if (ui::Button(ICON_FA_PLUS, "生成交通", ui::Kind::Primary)) SpawnTraffic();
  ImGui::SameLine();
  if (ui::Button(ICON_FA_TRASH, "清除交通")) ClearTraffic();
  ImGui::EndDisabled();
  if (!traffic_count_.empty()) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
    ui::Pill(Fmt(ICON_FA_CAR " %d 辆车   " ICON_FA_PERSON_WALKING " %d 个行人", traffic_count_.value("vehicles", 0),
                 traffic_count_.value("walkers", 0)).c_str(), p.accent);
  }
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelActors() {
  const ui::Palette& p = ui::Colors();
  ui::BeginCard(ICON_FA_LAYER_GROUP, "场景对象");
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
  InputStr("##filter", actor_filter_);
  ImGui::SameLine();
  if (ui::Button(ICON_FA_ROTATE, "刷新")) RefreshActors();
  ImGui::SameLine();
  for (const char* f : {"*", "vehicle.*", "walker.*", "sensor.*"}) {
    if (ImGui::SmallButton(f)) { actor_filter_ = f; RefreshActors(); }
    ImGui::SameLine();
  }
  ImGui::NewLine();
  ImGui::TextColored(p.text_dim, "共 %d 个对象", static_cast<int>(actors_.size()));
  if (ImGui::BeginTable("actors", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_ScrollY |
                                         ImGuiTableFlags_SizingStretchProp, ImVec2(0, ImGui::GetFontSize() * 26))) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("ID");
    ImGui::TableSetupColumn("类型", ImGuiTableColumnFlags_WidthStretch, 3.0f);
    ImGui::TableSetupColumn("角色");
    ImGui::TableSetupColumn("位置 (m)", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("");
    ImGui::TableHeadersRow();
    int destroy = -1;
    for (const json& a : actors_) {
      ImGui::TableNextRow();
      const int id = a.value("id", 0);
      const std::string t = a.value("type_id", std::string());
      const char* icon = t.rfind("vehicle.", 0) == 0 ? ICON_FA_CAR : t.rfind("walker.", 0) == 0 ? ICON_FA_PERSON_WALKING
                       : t.rfind("sensor.", 0) == 0 ? ICON_FA_SATELLITE_DISH : ICON_FA_CIRCLE_DOT;
      ImGui::TableSetColumnIndex(0); ImGui::Text("%d", id);
      ImGui::TableSetColumnIndex(1); ImGui::TextColored(p.text_dim, "%s", icon); ImGui::SameLine(); ImGui::TextUnformatted(t.c_str());
      ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(a.value("role", std::string()).c_str());
      ImGui::TableSetColumnIndex(3); ImGui::Text("%.1f, %.1f, %.1f", a.value("x", 0.0), a.value("y", 0.0), a.value("z", 0.0));
      ImGui::TableSetColumnIndex(4); if (a.value("ego", false)) ui::Pill("主车", p.success);
      ImGui::TableSetColumnIndex(5);
      if (ui::IconButton(ICON_FA_TRASH, "删除", Fmt("del%d", id).c_str())) destroy = id;
    }
    ImGui::EndTable();
    if (destroy >= 0) Call("destroy_actor", {{"id", destroy}}, [this](const json&) { RefreshActors(); RefreshWorld(); });
  }
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelVehicle() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ui::BeginCard(ICON_FA_CAR_SIDE, "车型");
  ImGui::SetNextItemWidth(fs * 12);
  InputStr("##vf", vehicle_filter_);
  if (vehicle_filter_.empty()) {
    ImGui::SameLine(fs * 1.2f);
    ImGui::TextColored(p.text_dim, "搜索车型 ...");
  }
  ImGui::SameLine(fs * 13.5f);
  if (ui::Button(ICON_FA_ROTATE, "刷新")) RefreshVehicles();
  ImGui::SameLine();
  ImGui::BeginDisabled(!busy_.empty());
  if (ui::Button(ICON_FA_RULER, "测量全部车型尺寸")) FetchVehicleSpecs(true);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ui::HelpMarker("逐个在高空临时生成一次，测量轮胎半径、轴距、轮距、质量。用来挑选和 CarSim 模型尺寸接近的车型");
  if (ImGui::BeginTable("vehicles", 8, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_ScrollY |
                                           ImGuiTableFlags_SizingStretchProp, ImVec2(0, fs * 13))) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("蓝图", ImGuiTableColumnFlags_WidthStretch, 3.0f);
    ImGui::TableSetupColumn("类型");
    ImGui::TableSetupColumn("轮胎半径 m");
    ImGui::TableSetupColumn("轴距 m");
    ImGui::TableSetupColumn("轮距 m");
    ImGui::TableSetupColumn("长 × 宽 × 高 m", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("质量 kg");
    ImGui::TableSetupColumn("最大转角 °");
    ImGui::TableHeadersRow();
    for (int i = 0; i < static_cast<int>(vehicles_.size()); ++i) {
      const json& v = vehicles_[static_cast<size_t>(i)];
      const std::string id = v.value("id", std::string());
      if (!vehicle_filter_.empty() && id.find(vehicle_filter_) == std::string::npos) continue;
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      if (ImGui::Selectable(id.c_str(), vehicle_sel_ == i, ImGuiSelectableFlags_SpanAllColumns)) {
        vehicle_sel_ = i;
        cfg_["carla"]["vehicle"] = id;
      }
      ImGui::TableSetColumnIndex(1); ImGui::TextColored(p.text_dim, "%s", v.value("base_type", std::string()).c_str());
      const json& s = v.contains("spec") ? v["spec"] : json();
      if (s.is_object()) {
        const json wr = s.value("wheel_radius_m", json::array()), ms = s.value("max_steer_deg", json::array());
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.3f", wr.empty() ? 0.0 : appui::NumAt(wr, 0));
        ImGui::TableSetColumnIndex(3); ImGui::Text("%.3f", s.value("wheelbase_m", 0.0));
        ImGui::TableSetColumnIndex(4); ImGui::Text("%.3f", s.value("track_m", 0.0));
        ImGui::TableSetColumnIndex(5); ImGui::Text("%.2f × %.2f × %.2f", s.value("length_m", 0.0), s.value("width_m", 0.0), s.value("height_m", 0.0));
        ImGui::TableSetColumnIndex(6); ImGui::Text("%.0f", s.value("mass_kg", 0.0));
        ImGui::TableSetColumnIndex(7); ImGui::Text("%.0f", ms.empty() ? 0.0 : appui::NumAt(ms, 0));
      } else {
        for (int c = 2; c < 8; ++c) { ImGui::TableSetColumnIndex(c); ImGui::TextColored(p.text_dim, "-"); }
      }
    }
    ImGui::EndTable();
  }
  ui::EndCard();

  ui::BeginCard(ICON_FA_LOCATION_DOT, "出生点（也是 CarSim 坐标原点）");
  const int n = static_cast<int>(spawn_points_.size());
  int sp = cfg_["carla"].value("spawn_index", 0);
  const float map_w = ImGui::GetContentRegionAvail().x * 0.55f;
  ImGui::BeginGroup();
  if (n && ImPlot::BeginPlot("##spawnmap", ImVec2(map_w, fs * 16), ImPlotFlags_Equal | ImPlotFlags_NoMenus | ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("x (m)", "y (m)", ImPlotAxisFlags_None, ImPlotAxisFlags_Invert);
    std::vector<float> xs(static_cast<size_t>(n)), ys(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      xs[static_cast<size_t>(i)] = spawn_points_[static_cast<size_t>(i)].value("x", 0.0f);
      ys[static_cast<size_t>(i)] = spawn_points_[static_cast<size_t>(i)].value("y", 0.0f);
    }
    ImPlotSpec all;
    all.Marker = ImPlotMarker_Circle;
    all.MarkerSize = 2.5f;
    all.MarkerFillColor = ImVec4(0.55f, 0.6f, 0.7f, 0.8f);
    all.MarkerLineColor = ImVec4(0, 0, 0, 0);
    ImPlot::PlotScatter("出生点", xs.data(), ys.data(), n, all);
    if (sp >= 0 && sp < n) {
      ImPlotSpec s;
      s.Marker = ImPlotMarker_Diamond;
      s.MarkerSize = 7.0f;
      s.MarkerFillColor = p.accent;
      s.MarkerLineColor = ImVec4(1, 1, 1, 1);
      s.LineWeight = 1.5f;
      ImPlot::PlotScatter("选中", &xs[static_cast<size_t>(sp)], &ys[static_cast<size_t>(sp)], 1, s);
    }
    if (!last_tel_.empty() && Running()) {
      float ex = static_cast<float>(appui::NumAt(last_tel_["location"], 0)), ey = static_cast<float>(appui::NumAt(last_tel_["location"], 1));
      ImPlotSpec s;
      s.Marker = ImPlotMarker_Square;
      s.MarkerSize = 6.0f;
      s.MarkerFillColor = p.success;
      s.MarkerLineColor = ImVec4(1, 1, 1, 1);
      ImPlot::PlotScatter("主车", &ex, &ey, 1, s);
    }
    if (ImPlot::IsPlotHovered() && ImGui::IsMouseClicked(0)) {
      ImPlotPoint m = ImPlot::GetPlotMousePos();
      double best = 1e18;
      for (int i = 0; i < n; ++i) {
        const double d = std::hypot(xs[static_cast<size_t>(i)] - m.x, ys[static_cast<size_t>(i)] - m.y);
        if (d < best) { best = d; sp = i; }
      }
      cfg_["carla"]["spawn_index"] = sp;
    }
    ImPlot::EndPlot();
  }
  ImGui::EndGroup();
  ImGui::SameLine();
  ImGui::BeginGroup();
  ImGui::TextColored(p.text_dim, "点击地图选择出生点");
  ImGui::SetNextItemWidth(fs * 8);
  if (ImGui::InputInt("编号", &sp)) cfg_["carla"]["spawn_index"] = std::max(0, std::min(sp, std::max(0, n - 1)));
  sp = cfg_["carla"].value("spawn_index", 0);
  if (sp >= 0 && sp < n) {
    const json& q = spawn_points_[static_cast<size_t>(sp)];
    ImGui::Text("x %.1f   y %.1f   z %.1f", q.value("x", 0.0), q.value("y", 0.0), q.value("z", 0.0));
    ImGui::Text("航向 %.0f°   （共 %d 个）", q.value("yaw", 0.0), n);
  }
  ImGui::Dummy(ImVec2(0, fs * 0.5f));
  ImGui::Checkbox("自定义车身颜色", &ego_custom_color_);
  if (ego_custom_color_) {
    ImGui::SetNextItemWidth(fs * 10);
    ImGui::ColorEdit3("##color", ego_color_, ImGuiColorEditFlags_NoInputs);
  }
  const std::string sel = vehicle_sel_ >= 0 && vehicle_sel_ < static_cast<int>(vehicles_.size())
                              ? vehicles_[static_cast<size_t>(vehicle_sel_)].value("id", std::string()) : std::string("（未选择）");
  ImGui::TextColored(p.text_dim, "车型：%s", sel.c_str());
  ImGui::BeginDisabled(!busy_.empty() || Running());
  if (ui::Button(ICON_FA_PLUS, world_.value("ego_id", 0) ? "更换主车" : "生成主车", ui::Kind::Primary)) SpawnEgo();
  ImGui::SameLine();
  if (ui::Button(ICON_FA_TRASH, "删除主车")) DestroyEgo();
  ImGui::EndDisabled();
  if (world_.value("ego_id", 0)) {
    ImGui::BeginDisabled(Running());
    if (ImGui::Checkbox("用 CARLA 自动驾驶试开", &autopilot_)) Call("ego_autopilot", {{"enabled", autopilot_}}, nullptr);
    ImGui::EndDisabled();
  }
  ImGui::EndGroup();
  ui::EndCard();

  ui::BeginCard(ICON_FA_EYE, "服务器观察视角");
  const std::string mode = world_.value("spectator_mode", std::string("free"));
  const char* modes[] = {"free", "follow", "top", "side"};
  const char* names[] = {"自由", "跟车", "俯视", "侧面"};
  for (int i = 0; i < 4; ++i) {
    if (i) ImGui::SameLine();
    if (ImGui::RadioButton(names[i], mode == modes[i])) SetSpectator(modes[i]);
  }
  ImGui::SameLine();
  ui::HelpMarker("控制 CARLA 服务器窗口的观察相机。服务器离屏运行时，请用“实时画面”页在界面里看");
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelDrive() {
  if (!cfg_.contains("drive")) { ImGui::TextDisabled("等待后端返回配置 ..."); return; }
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  json& dr = cfg_["drive"];
  if (!dr.contains("cosim_driver")) dr["cosim_driver"] = cfg_["run"].value("driver", std::string("custom"));
  const bool cosim = dr.value("dynamics", std::string("cosim")) == "cosim";

  ui::BeginCard(ICON_FA_GEARS, "车辆动力学由谁计算");
  const float w2 = ImGui::GetContentRegionAvail().x;
  ImGui::BeginDisabled(Running());
  if (ChoiceCard("dyn_cosim", ICON_FA_GEARS, "CarSim 联合仿真", "你的控制算法控制 CarSim 的车，CarSim 计算动力学；CARLA 照 CarSim 的结果摆放车辆，负责场景、传感器和画面。", cosim, w2))
    dr["dynamics"] = "cosim";
  if (ChoiceCard("dyn_carla", ICON_FA_CAR, "CARLA 物理", "CARLA 自带 PhysX 车辆物理，不需要 CarSim。适合大规模数据采集、感知算法。", !cosim, w2))
    dr["dynamics"] = "carla";
  ImGui::EndDisabled();
  ui::EndCard();

  struct D { const char* id; const char* icon; const char* title; const char* desc; };
  const std::string key = cosim ? "cosim_driver" : "carla_driver";
  std::string cur = dr.value(key, std::string(cosim ? "custom" : "route"));
  const float w3 = ImGui::GetContentRegionAvail().x;
  auto cards = [&](const D* list, int n) {
    ImGui::BeginDisabled(Running());
    for (int i = 0; i < n; ++i) {
      if (ChoiceCard(list[i].id, list[i].icon, list[i].title, list[i].desc, cur == list[i].id, w3)) dr[key] = cur = list[i].id;
    }
    ImGui::EndDisabled();
  };

  if (cosim) {
    ui::BeginCard(ICON_FA_CODE, "控制算法");
    ImGui::BeginDisabled(Running());
    if (ChoiceCard("custom", ICON_FA_CODE, "我的控制算法", "每帧把 CarSim 的导出变量交给你的 Python 算法，算出的油门、制动、方向盘送回 CarSim（REPLACE 导入）。",
                   cur == "custom", ImGui::GetContentRegionAvail().x))
      dr[key] = cur = "custom";
    ImGui::EndDisabled();
    if (cur == "custom") {
      json& ctl = cfg_["run"]["controller"];
      ImGui::Dummy(ImVec2(0, fs * 0.2f));
      ui::Row("算法文件 .py", "相对路径以 carsim_carla_bridge 目录为准。每次点“运行”都会重新加载，改完代码直接再点运行", fs * 30);
      EditString(ctl, "path");
      ui::Row("入口", "文件里的类名（带 control 方法）或函数名", fs * 8);
      EditString(ctl, "entry");
      ui::DimWrapped("接口：control(exports, t, dt) -> [油门, 制动, 方向盘角]，exports 是按变量名取值的 CarSim 导出变量");
      ui::DimWrapped("示例：controllers/example_controller.py（定速 + 蛇形），controllers/simple_path_follower.py（SimplePathFollower）");
    }
    ImGui::Dummy(ImVec2(0, fs * 0.3f));
    if (ui::FoldHeader(ICON_FA_FLASK, "测试用驾驶方式（还没有算法时，用来检查联合仿真链路）", false, cur != "custom")) {
      static const D kTest[] = {
          {"demo", ICON_FA_WAVE_SQUARE, "演示（开环）", "固定的加速 + 蛇形 + 制动，看 CARLA 的车是否跟着 CarSim 动。"},
          {"route", ICON_FA_ROUTE, "路线跟随", "程序在 CARLA 路网上规划路线，算出油门、方向盘送给 CarSim。"},
          {"manual", ICON_FA_KEYBOARD, "键盘驾驶", "W/S 油门刹车，A/D 转向，手动开 CarSim 的车。"}};
      cards(kTest, 3);
    }
  } else {
    ui::BeginCard(ICON_FA_ROBOT, "驾驶方式");
    static const D kCarla[] = {
        {"route", ICON_FA_ROUTE, "路线跟随", "在路网上规划路线，自动转向、控速，走完一段自动换下一个目的地。"},
        {"autopilot", ICON_FA_TRAFFIC_LIGHT, "CARLA 自动驾驶", "交通管理器驾驶：遵守信号灯、跟车、变道。"},
        {"manual", ICON_FA_KEYBOARD, "键盘驾驶", "W/S 油门刹车，A/D 转向，在界面里直接开。"}};
    cards(kCarla, 3);
  }
  ImGui::Dummy(ImVec2(0, fs * 0.3f));
  if (cur == "route") {
    float v = dr.value("target_speed_kmh", 40.0f);
    ui::Row("目标车速 km/h");
    if (ImGui::SliderFloat("##tspeed", &v, 5, 120, "%.0f")) dr["target_speed_kmh"] = v;
    int dest = dr.value("destination_index", -1);
    ui::Row("目的地出生点", "-1 = 随机选择远处目的地，走完自动换下一个", fs * 8);
    if (ImGui::InputInt("##dest", &dest)) dr["destination_index"] = std::max(-1, dest);
  } else if (cur == "autopilot") {
    float d = dr.value("tm_speed_diff_pct", 0.0f);
    ui::Row("相对限速 %", "负数 = 比限速快，正数 = 比限速慢");
    if (ImGui::SliderFloat("##tmsd", &d, -50, 80, "%.0f %%")) dr["tm_speed_diff_pct"] = d;
    bool ign = dr.value("tm_ignore_lights", false);
    ui::Row("忽略红绿灯");
    if (ImGui::Checkbox("##ign", &ign)) dr["tm_ignore_lights"] = ign;
  }
  if (cosim && (cur == "route" || cur == "manual")) {
    float b = dr.value("brake_scale", 1.0f);
    ui::Row("制动输入比例", "0..1 的制动指令乘以这个系数再送给 CarSim（例如 CarSim 用制动压力 MPa 时设为 10）", fs * 8);
    if (ImGui::InputFloat("##bscale", &b, 0.5f, 1.0f, "%.2f")) dr["brake_scale"] = std::max(0.0f, b);
  }
  ui::EndCard();

  if (cur == "manual") {
    ui::BeginCard(ICON_FA_KEYBOARD, "键盘驾驶");
    const bool live = run_state_ == "running";
    ImGui::TextColored(live ? p.success : p.text_dim, "%s", live ? "正在键盘驾驶：直接按键即可（输入框获得焦点时不响应）"
                                                                 : "点顶部“运行”后，用下面的按键驾驶");
    ImGui::BeginGroup();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + fs * 2.4f);
    KeyCap("W", live && (ImGui::IsKeyDown(ImGuiKey_W) || ImGui::IsKeyDown(ImGuiKey_UpArrow)));
    KeyCap("A", live && (ImGui::IsKeyDown(ImGuiKey_A) || ImGui::IsKeyDown(ImGuiKey_LeftArrow)));
    ImGui::SameLine(0, fs * 0.4f);
    KeyCap("S", live && (ImGui::IsKeyDown(ImGuiKey_S) || ImGui::IsKeyDown(ImGuiKey_DownArrow)));
    ImGui::SameLine(0, fs * 0.4f);
    KeyCap("D", live && (ImGui::IsKeyDown(ImGuiKey_D) || ImGui::IsKeyDown(ImGuiKey_RightArrow)));
    ImGui::EndGroup();
    ImGui::SameLine(0, fs * 2);
    ImGui::BeginGroup();
    ImGui::TextColored(p.text_dim, "W / ↑  油门");
    ImGui::TextColored(p.text_dim, "S / ↓  制动");
    ImGui::TextColored(p.text_dim, "A D / ← →  转向");
    ImGui::TextColored(p.text_dim, "输入会平滑变化，像真实的踏板和方向盘");
    ImGui::Text("油门 %.2f   制动 %.2f   转向 %+.2f", kb_throttle_, kb_brake_, kb_steer_);
    ImGui::EndGroup();
    ui::EndCard();
  }

  ui::BeginCard(ICON_FA_CLOCK, "运行设置");
  ui::Row("仿真步长 s", "CARLA 每帧的仿真时间。CarSim 每帧内部积分 步长 / t_step 步", fs * 8);
  EditDouble(cfg_["sync"], "frame_dt", 0.005, "%.3f", 0.001, 0.5);
  ui::Row("运行时长 s", "0 = 一直运行，直到点“停止”（默认）。设了时长，到时会自动结束并停车", fs * 8);
  EditDouble(cfg_["sync"], "duration", 1.0, "%.1f", 0.0, 1e6);
  ui::Row("日志 CSV", "每帧的 CarSim / CARLA 位姿对比，空 = 不记录");
  EditString(cfg_["run"], "log_path");
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelCoSim() {
  if (!cfg_.contains("carsim")) { ImGui::TextDisabled("等待后端返回配置 ..."); return; }
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  json& cs = cfg_["carsim"];
  json& sy = cfg_["sync"];
  static const std::vector<std::string> kRequired = {"Xo", "Yo", "Zo", "Yaw", "Pitch", "Roll", "Steer_L1", "Steer_R1"};
  static const std::vector<std::string> kRecommended = {
      "Vx", "Vy", "AVx", "AVy", "AVz", "Steer_SW", "Steer_L2", "Steer_R2", "AVy_L1", "AVy_R1",
      "AVy_L2", "AVy_R2", "Throttle", "GearStat", "Jnc_L1", "Jnc_R1", "Jnc_L2", "Jnc_R2"};

  if (!(cfg_.contains("drive") && cfg_["drive"].value("dynamics", std::string("cosim")) == "cosim")) {
    ImGui::TextColored(p.warning, ICON_FA_CIRCLE_INFO "  当前在“驾驶模式”页选择的是 CARLA 物理，这里的设置只在 CarSim 联合仿真下生效。");
    ImGui::Dummy(ImVec2(0, 4));
  }

  ui::BeginCard(ICON_FA_FOLDER_OPEN, "CarSim 模型");
  bool mock = cs.value("mock", false);
  ui::Row("模拟 CarSim", "不需要 CarSim 许可证，用一个简单车辆模型代替，用来测试整条链路");
  if (ImGui::Checkbox("##mock", &mock)) cs["mock"] = mock;
  ImGui::BeginDisabled(mock);
  ui::Row(".sim 文件");
  EditString(cs, "sim_path");
  ui::Row("python_carsim_env 目录");
  EditString(cs, "repo_path");
  ImGui::EndDisabled();
  ui::EndCard();

  ui::BeginCard(ICON_FA_LIST, "导出变量（顺序必须与 .sim 一致）");
  json& names = cs["export_names"];
  std::vector<std::string> missing;
  for (const auto& r : kRequired) {
    bool found = false;
    for (const auto& nm : names) found |= nm.get<std::string>() == r;
    if (!found) missing.push_back(r);
  }
  if (missing.empty()) {
    ui::Pill(Fmt(ICON_FA_CIRCLE_CHECK " 必需变量齐全 · 共 %d 个", static_cast<int>(names.size())).c_str(), p.success);
  } else {
    std::string m;
    for (const auto& x : missing) m += x + " ";
    ui::Pill((ICON_FA_TRIANGLE_EXCLAMATION " 缺少：" + m).c_str(), p.danger);
  }
  // Editor beside the list when there is room, below it in a narrow panel.
  const bool narrow = ImGui::GetContentRegionAvail().x < fs * 38;
  ImGui::BeginChild("exports", ImVec2(narrow ? 0.0f : fs * 19, fs * 15), ImGuiChildFlags_Borders);
  int mv_from = -1, mv_to = -1, erase = -1;
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    const std::string nm = names[static_cast<size_t>(i)];
    const bool req = std::find(kRequired.begin(), kRequired.end(), nm) != kRequired.end();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(p.text_dim, "%2d", i);
    ImGui::SameLine();
    if (req) ImGui::TextColored(p.accent, "%s", nm.c_str()); else ImGui::TextUnformatted(nm.c_str());
    ImGui::SameLine(fs * 11.5f);
    if (ui::IconButton(ICON_FA_ARROW_UP, "上移", Fmt("u%d", i).c_str()) && i > 0) { mv_from = i; mv_to = i - 1; }
    ImGui::SameLine(0, 0);
    if (ui::IconButton(ICON_FA_ARROW_DOWN, "下移", Fmt("d%d", i).c_str()) && i + 1 < static_cast<int>(names.size())) { mv_from = i; mv_to = i + 1; }
    ImGui::SameLine(0, 0);
    if (ui::IconButton(ICON_FA_XMARK, "删除", Fmt("x%d", i).c_str())) erase = i;
  }
  if (mv_from >= 0) std::swap(names[static_cast<size_t>(mv_from)], names[static_cast<size_t>(mv_to)]);
  if (erase >= 0) names.erase(names.begin() + erase);
  ImGui::EndChild();
  if (!narrow) ImGui::SameLine();
  ImGui::BeginGroup();
  ImGui::SetNextItemWidth(fs * 9);
  InputStr("##newexp", new_export_);
  ImGui::SameLine();
  if (ui::Button(ICON_FA_PLUS, "添加") && !new_export_.empty()) { names.push_back(new_export_); new_export_.clear(); }
  ImGui::TextColored(p.text_dim, "从 CarSim 复制导出列表粘贴到这里：");
  appui::InputStrMultiline("##paste", paste_exports_, fs * 16, fs * 4);
  if (ui::Button(ICON_FA_PASTE, "用粘贴内容替换")) {
    json out = json::array();
    std::string cur;
    for (char c : paste_exports_ + " ") {
      if (c == ' ' || c == '\n' || c == '\r' || c == ',' || c == '\t' || c == ';') {
        if (!cur.empty()) out.push_back(cur);
        cur.clear();
      } else {
        cur += c;
      }
    }
    if (!out.empty()) names = out;
  }
  if (ui::Button(ICON_FA_WAND_MAGIC_SPARKLES, "补全推荐变量")) {
    for (const auto& r : kRecommended) {
      bool found = false;
      for (const auto& nm : names) found |= nm.get<std::string>() == r;
      if (!found) names.push_back(r);
    }
  }
  ImGui::SameLine();
  ui::HelpMarker("追加的变量也要在 CarSim 的 Export 列表里按同样顺序加上。蓝色 = 必需变量");
  ImGui::EndGroup();
  ui::EndCard();

  ui::BeginCard(ICON_FA_RULER, "单位与坐标");
  json& u = cs["units"];
  struct U { const char* key; const char* name; std::vector<std::string> opts; };
  const U kU[] = {{"angle", "角度", {"deg", "rad"}}, {"speed", "速度", {"km/h", "m/s"}}, {"rate", "角速度", {"deg/s", "rad/s"}},
                  {"wheel_spin", "车轮转速", {"rpm", "rad/s"}}, {"jounce", "悬架行程", {"mm", "m"}}};
  for (const auto& x : kU) {
    std::string v = u.value(x.key, x.opts[0]);
    ui::Row(x.name, nullptr, fs * 8);
    if (ComboStr(Fmt("##u%s", x.key).c_str(), v, x.opts)) u[x.key] = v;
  }
  std::string zm = sy.value("z_mode", std::string("carsim"));
  const std::vector<std::string> zmodes = {"carsim", "ground"}, znames = {"使用 CarSim 高度", "贴合 CARLA 路面"};
  ui::Row("高度模式", "CarSim 路面与 CARLA 地图高度不一致时选“贴合 CARLA 路面”", fs * 12);
  if (ComboStr("##zmode", zm, zmodes, &znames)) sy["z_mode"] = zm;
  // A reference point is "front_axle" or [x, y, z]; anything else counts as front axle.
  const json& rp = sy["reference_point"];
  const bool rp_ok = rp.is_array() && rp.size() >= 3 && rp[0].is_number() && rp[1].is_number() && rp[2].is_number();
  bool front_axle = !rp_ok;
  ui::Row("参考点在前轴中心", "CarSim 的 Xo/Yo/Zo 默认是前轴中心地面处；不是的话取消勾选并填写偏移");
  if (ImGui::Checkbox("##refpt", &front_axle))
    sy["reference_point"] = front_axle ? json("front_axle") : json::array({1.5, 0.0, 0.0});
  if (!front_axle) {
    float pt[3] = {rp_ok ? static_cast<float>(appui::NumAt(rp, 0)) : 1.5f, rp_ok ? static_cast<float>(appui::NumAt(rp, 1)) : 0.0f, rp_ok ? static_cast<float>(appui::NumAt(rp, 2)) : 0.0f};
    ui::Row("参考点 x/y/z m", nullptr, fs * 14);
    if (ImGui::InputFloat3("##refxyz", pt, "%.3f")) sy["reference_point"] = {pt[0], pt[1], pt[2]};
  }
  const json& ea = sy["use_external_api"];
  std::string ext = ea.is_boolean() ? (ea.get<bool>() ? "on" : "off") : std::string("auto");
  const std::vector<std::string> exts = {"auto", "on", "off"}, extn = {"自动", "强制改版接口", "强制原版兼容"};
  ui::Row("CARLA 接口", nullptr, fs * 12);
  if (ComboStr("##ext", ext, exts, &extn)) sy["use_external_api"] = ext == "auto" ? json("auto") : json(ext == "on");
  ui::Row("方向盘满量程 °", "方向盘转这么多度对应 CARLA 的 steer = ±1", fs * 8);
  EditDouble(sy, "steering_wheel_max_deg", 10, "%.0f", 1, 3600);
  ui::EndCard();

  ui::BeginCard(ICON_FA_FLOPPY_DISK, "配置文件");
  static std::string path, shown_cfg;
  if (path.empty() || shown_cfg != cfg_path_) {  // follow Ctrl+S / loads
    path = cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_;
    shown_cfg = cfg_path_;
  }
  ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - fs * 16);
  InputStr("##cfgpath", path);
  ImGui::SameLine();
  if (ui::Button(ICON_FA_FOLDER_OPEN, "载入")) LoadConfig(path);
  ImGui::SameLine();
  if (ui::Button(ICON_FA_FLOPPY_DISK, "保存")) SaveConfig(path);
  ImGui::SameLine();
  if (ui::Button(ICON_FA_ROTATE_LEFT, "默认"))
    Call("default_config", json::object(), [this](const json& r) { json carla = cfg_["carla"]; cfg_ = r; cfg_["carla"].update(carla); Log("已恢复默认配置"); });
  ui::DimWrapped("同一个 JSON 也能给命令行和强化学习训练用：python run_cosim.py --config <文件>");
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelCollect() {
  if (!cfg_.contains("collect")) { ImGui::TextDisabled("等待后端返回配置 ..."); return; }
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  json& c = cfg_["collect"];

  ui::BeginCard(ICON_FA_DATABASE, "数据采集");
  bool en = c.value("enabled", false);
  ui::Row("运行时采集数据", "打开后，点顶部“运行”会同时按“传感器套件”页的配置同步采集");
  ImGui::BeginDisabled(Running());
  if (ImGui::Checkbox("##en", &en)) {
    c["enabled"] = en;
    // A 50 Hz co-sim step would mean 5x the data of a typical 10 Hz dataset.
    if (en && cfg_["sync"].value("frame_dt", 0.1) < 0.05 - 1e-9) {
      cfg_["sync"]["frame_dt"] = 0.1;
      Log("采集已开启：采集频率设为 10 Hz（常见数据集的频率），可在下面修改", "warn");
    }
    RefreshDisk();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ui::Pill(en ? "开启" : "关闭", en ? p.success : p.text_dim);
  ui::Row("输出目录");
  if (EditString(c, "out_dir")) RefreshDisk();
  ui::Row("场景名称", "空 = 按时间自动命名，例如 session_20260925_143000");
  EditString(c, "session");
  ui::EndCard();

  ui::BeginCard(ICON_FA_SLIDERS, "采集内容与格式");
  const double dt = cfg_["sync"].value("frame_dt", 0.1);
  float hz = static_cast<float>(1.0 / std::max(1e-3, dt));
  ui::Row("采集频率 Hz", "每一帧所有传感器同时采样（同一时间戳）。同时决定仿真步长 = 1 / 频率", fs * 10);
  if (ImGui::SliderFloat("##hz", &hz, 1, 50, "%.0f Hz")) cfg_["sync"]["frame_dt"] = 1.0 / std::max(1.0f, hz);
  if (hz > 20.5f) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
    ImGui::TextColored(p.warning, ICON_FA_TRIANGLE_EXCLAMATION "  %.0f Hz 数据量很大；KITTI / nuScenes 等数据集一般用 10 Hz", hz);
  }
  int every = c.value("capture_every", 1);
  ui::Row("每 N 帧存一次", "仿真频率高但不需要每帧都存时用，例如 20 Hz 仿真、每 2 帧存 = 10 Hz 数据", fs * 8);
  if (ImGui::InputInt("##every", &every)) c["capture_every"] = std::max(1, every);
  std::string imf = c.value("image_format", std::string("jpg"));
  const std::vector<std::string> imfs = {"jpg", "png"}, imfn = {"JPG（小，约 1/4 大小）", "PNG（无损）"};
  ui::Row("RGB 图像格式", "深度、语义、实例图始终是无损 PNG", fs * 14);
  if (ComboStr("##imf", imf, imfs, &imfn)) c["image_format"] = imf;
  if (imf == "jpg") {
    int q = c.value("jpg_quality", 90);
    ui::Row("JPG 质量", nullptr, fs * 10);
    if (ImGui::SliderInt("##q", &q, 50, 100)) c["jpg_quality"] = q;
  }
  std::string pcf = c.value("pointcloud_format", std::string("bin"));
  const std::vector<std::string> pcfs = {"bin", "npy"}, pcfn = {".bin（KITTI 格式 float32 x,y,z,强度）", ".npy（NumPy）"};
  ui::Row("点云格式", nullptr, fs * 16);
  if (ComboStr("##pcf", pcf, pcfs, &pcfn)) c["pointcloud_format"] = pcf;
  bool labels = c.value("labels", true);
  ui::Row("真值标注", "每帧保存周围车辆和行人的 3D 框（主车坐标系）、类别、ID、速度");
  if (ImGui::Checkbox("##labels", &labels)) c["labels"] = labels;
  if (labels) {
    ImGui::SameLine();
    float r = c.value("label_radius", 80.0f);
    ImGui::SetNextItemWidth(fs * 8);
    if (ImGui::SliderFloat("范围 m", &r, 20, 200, "%.0f")) c["label_radius"] = r;
  }
  ui::EndCard();

  ui::BeginCard(ICON_FA_TRIANGLE_EXCLAMATION, "停止条件（至少设置一项，防止写满磁盘）");
  int mf = c.value("max_frames", 100);
  ui::Row("最多帧数", "0 = 不限", fs * 8);
  if (ImGui::InputInt("##mf", &mf, 10, 100)) c["max_frames"] = std::max(0, mf);
  float ms = c.value("max_seconds", 0.0f);
  ui::Row("最长时间 s", "0 = 不限（真实时间）", fs * 8);
  if (ImGui::InputFloat("##ms", &ms, 10, 60, "%.0f")) c["max_seconds"] = std::max(0.0f, ms);
  float mg = c.value("max_gb", 2.0f);
  ui::Row("最大容量 GB", "0 = 不限。磁盘剩余少于 10 GB 时也会自动停止", fs * 8);
  if (ImGui::InputFloat("##mg", &mg, 0.5f, 5.0f, "%.1f")) c["max_gb"] = std::max(0.0f, mg);
  if (mf == 0 && ms <= 0 && mg <= 0) ImGui::TextColored(p.danger, ICON_FA_TRIANGLE_EXCLAMATION "  没有任何停止条件，后端会拒绝开始采集");
  ui::EndCard();

  DrawRigEstimate(false);

  ui::BeginCard(ICON_FA_FOLDER_OPEN, "输出结构");
  const std::string root = c.value("out_dir", std::string("datasets")) + "/" +
                           (c.value("session", std::string()).empty() ? std::string("session_日期_时间") : c.value("session", std::string()));
  ImGui::TextColored(p.accent, ICON_FA_FOLDER_OPEN " %s/", root.c_str());
  const json& sensors = RigSensors();
  auto line = [&](const char* icon, const std::string& name, const char* note) {
    ImGui::TextColored(p.text_dim, "    %s", icon);
    ImGui::SameLine();
    ImGui::TextUnformatted(name.c_str());
    ImGui::SameLine(fs * 16);
    ImGui::TextColored(p.text_dim, "%s", note);
  };
  line(ICON_FA_FILE_CODE, "meta.json", "地图、天气、套件、采集设置");
  line(ICON_FA_FILE_CODE, "calib.json", "每个传感器的外参（传感器→车辆 4×4）与相机内参 K");
  for (const json& s : sensors) {
    if (!s.value("enabled", true)) continue;
    const std::string t = s.value("type", std::string());
    const char* note = t == "rgb" ? (imf == "jpg" ? "000123.jpg" : "000123.png")
                     : t == "lidar" ? (pcf == "bin" ? "000123.bin" : "000123.npy")
                     : t == "radar" ? "000123.csv" : (t == "imu" || t == "gnss") ? "写入 ego/*.json" : "000123.png";
    line(t == "lidar" ? ICON_FA_CIRCLE_NODES : t == "radar" ? ICON_FA_WIFI : ICON_FA_CAMERA, s.value("name", std::string()) + "/", note);
  }
  if (labels) line(ICON_FA_TAGS, "labels/", "000123.json  3D 框、类别、ID、速度");
  line(ICON_FA_CAR, "ego/", "000123.json  位姿、速度、加速度、控制量、CarSim 状态");
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelRecorder() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ui::BeginCard(ICON_FA_CIRCLE_DOT, "录制");
  ui::DimWrapped("CARLA 录制器记录整个场景（所有车辆、行人、信号灯），可以之后换一套传感器重新回放采集。");
  ui::Row("文件", "保存在 CARLA 服务器所在机器上");
  InputStr("##rec", rec_file_);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  ImGui::BeginDisabled(recording_);
  if (ui::Button(ICON_FA_CIRCLE_DOT, "开始录制", ui::Kind::Danger))
    Call("start_recorder", {{"filename", rec_file_}}, [this](const json&) { recording_ = true; });
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!recording_);
  if (ui::Button(ICON_FA_STOP, "停止录制")) Call("stop_recorder", json::object(), [this](const json&) { recording_ = false; });
  ImGui::EndDisabled();
  if (recording_) { ImGui::SameLine(); ui::Pill(ICON_FA_CIRCLE " 录制中", p.danger); }
  ui::EndCard();

  ui::BeginCard(ICON_FA_FILM, "回放");
  ui::Row("起始时间 s", nullptr, fs * 8);
  ImGui::InputFloat("##rs", &replay_start_, 0, 0, "%.1f");
  ui::Row("时长 s", "0 = 全部", fs * 8);
  ImGui::InputFloat("##rd", &replay_duration_, 0, 0, "%.1f");
  ui::Row("相机跟随 ID", "0 = 不跟随", fs * 8);
  ImGui::InputInt("##rf", &replay_follow_, 0);
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  if (ui::Button(ICON_FA_PLAY, "回放", ui::Kind::Primary))
    Call("replay", {{"filename", rec_file_}, {"start", replay_start_}, {"duration", replay_duration_}, {"follow_id", replay_follow_}},
         [this](const json& r) {
           Log("回放：" + r.get<std::string>().substr(0, 200));
           RefreshWorld();  // the viewport follows the recorded ego
         });
  ImGui::SameLine();
  if (ui::Button(ICON_FA_CIRCLE_INFO, "文件信息"))
    Call("recorder_info", {{"filename", rec_file_}}, [this](const json& r) { rec_info_ = r.get<std::string>(); });
  if (!rec_info_.empty()) {
    ImGui::BeginChild("recinfo", ImVec2(0, fs * 14), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted(rec_info_.c_str());
    ImGui::EndChild();
  }
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawPanelView() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ui::BeginCard(ICON_FA_VIDEO, "主车相机画面");
  const char* modes[] = {"chase", "hood", "wheel", "top"};
  const char* names[] = {"跟车", "车头", "前轮特写", "俯视"};
  ui::Row("视角", "前轮特写可以直接看到转向、车轮转动和悬架跳动");
  for (int i = 0; i < 4; ++i) {
    if (i) ImGui::SameLine();
    if (ImGui::RadioButton(names[i], view_rig_sensor_.empty() && view_mode_ == modes[i])) {
      view_mode_ = modes[i];
      if (view_on_) StartView();
    }
  }
  // Preview any camera of the current rig through its exact mount.
  std::vector<std::string> cams;
  for (const json& s : RigSensors())
    if (s.value("type", std::string()) == "rgb") cams.push_back(s.value("name", std::string()));
  if (!cams.empty()) {
    ui::Row("套件相机", "用传感器套件里相机的实际安装位置和视场角预览");
    std::string pick = view_rig_sensor_.empty() ? std::string("（选择）") : view_rig_sensor_;
    if (ComboStr("##rigcam", pick, cams)) {
      for (const json& s : RigSensors())
        if (s.value("name", std::string()) == pick) {
          view_rig_sensor_ = pick;
          json m = json{{"x", s.value("x", 0.0)}, {"y", s.value("y", 0.0)}, {"z", s.value("z", 0.0)}, {"pitch", s.value("pitch", 0.0)}, {"yaw", s.value("yaw", 0.0)}, {"roll", s.value("roll", 0.0)}};
          const json attrs = s.value("attributes", json::object());
          StartView(m, attrs.is_object() ? attrs.value("fov", 90.0f) : 90.0f);
          view_rig_sensor_ = pick;
        }
    }
  }
  const char* res[] = {"480×270", "640×360", "960×540", "1280×720"};
  ui::Row("分辨率", nullptr, fs * 8);
  if (ImGui::Combo("##res", &view_res_, res, 4) && view_on_) SendViews();
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  ImGui::BeginDisabled(world_.value("ego_id", 0) == 0);
  if (!view_on_) {
    if (ui::Button(ICON_FA_PLAY, "打开画面", ui::Kind::Primary)) StartView();
  } else if (ui::Button(ICON_FA_STOP, "关闭画面")) {
    Call("view_stop", json::object(), nullptr);
    view_on_ = false;
  }
  ImGui::EndDisabled();
  if (world_.value("ego_id", 0) == 0) { ImGui::SameLine(); ImGui::TextColored(p.text_dim, "先在“车辆与视角”页生成主车"); }
  if (view_on_) { ImGui::SameLine(); ImGui::TextColored(p.text_dim, "%d×%d · 已接收 %d 帧", view_w_, view_h_, view_frames_); }
  ImGui::TextColored(p.text_dim, ICON_FA_CIRCLE_INFO "  画面显示在中间的视口里；视口左上角也可以直接切换视角和布局。");
  ui::EndCard();

  ui::BeginCard(ICON_FA_TABLE_CELLS_LARGE, "多视图");
  const char* layouts[] = {"单画面", "1 大 + 3 小", "2 × 2"};
  ui::Row("布局", "多个视图同时显示：相机、语义分割、深度、实例分割、激光雷达点云、毫米波雷达，或传感器套件里的任意传感器");
  for (int i = 0; i < 3; ++i) {
    if (i) ImGui::SameLine();
    if (ImGui::RadioButton(layouts[i], view_layout_ == i) && view_layout_ != i) {
      view_layout_ = i;
      if (view_on_) SendViews();
    }
  }
  if (view_layout_ > 0) {
    const auto sources = ViewSources();
    std::vector<std::string> ids, labels;
    for (const auto& src : sources) { ids.push_back(src.first); labels.push_back(src.second); }
    for (int i = 1; i < 4; ++i) {
      ui::Row(Fmt("视图 %d", i).c_str(), nullptr, fs * 16);
      std::string v = panes_[i].source;
      if (ComboStr(Fmt("##pane%d", i).c_str(), v, ids, &labels)) {
        panes_[i].source = v;
        if (view_on_) SendViews();
      }
      if (view_on_ && panes_[i].frames > 0) {
        ImGui::SameLine();
        ImGui::TextColored(p.text_dim, "%d×%d", panes_[i].w, panes_[i].h);
      }
    }
    ImGui::TextColored(p.text_dim, ICON_FA_CIRCLE_INFO "  前视的语义 / 深度 / 实例用车头相机位置；选“套件 · …”则用传感器套件里的实际安装位置和参数。");
    ImGui::TextColored(p.text_dim, ICON_FA_CIRCLE_INFO "  视图越多，服务器渲染和传输越重；2 × 2 时每个视图 640×360。");
  }
  ui::EndCard();
}
