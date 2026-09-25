// Window shell: menu bar, toolbar, project tree, workspace, monitor,
// output console and status bar, separated by draggable splitters.
#include <algorithm>
#include <cmath>

#include "app.h"
#include "imgui.h"
#include "implot.h"
#include "ui_kit.h"

using appui::Fmt;

namespace {

struct NavItem {
  int panel;
  const char* icon;
  const char* name;
  const char* desc;
};
struct NavGroup {
  const char* caption;
  const char* icon;
  std::vector<NavItem> items;
};

const std::vector<NavGroup>& Nav() {
  static const std::vector<NavGroup> g = {
      {"开始", ICON_FA_HOUSE, {{kPanelConnect, ICON_FA_PLUG, "连接", "连接 CARLA 服务器，管理 Python 后端进程。"}}},
      {"场景", ICON_FA_EARTH_ASIA,
       {{kPanelWorld, ICON_FA_MAP, "地图与天气", "切换地图，设置天气、光照和仿真参数。"},
        {kPanelTraffic, ICON_FA_TRAFFIC_LIGHT, "交通流", "生成由交通管理器控制的背景车辆和行人。"},
        {kPanelActors, ICON_FA_LAYER_GROUP, "场景对象", "查看和删除场景里的全部对象。"}}},
      {"车辆", ICON_FA_CAR,
       {{kPanelVehicle, ICON_FA_CAR_SIDE, "车辆与视角", "选择车型和出生点，生成主车，设置服务器观察视角。"},
        {kPanelRig, ICON_FA_SATELLITE_DISH, "传感器套件", "配置相机、激光雷达、毫米波雷达的数量、参数和安装位置。"}}},
      {"仿真", ICON_FA_GEARS,
       {{kPanelDrive, ICON_FA_ROBOT, "驾驶模式", "选择动力学来源和控制算法，设置运行参数。"},
        {kPanelCoSim, ICON_FA_GEARS, "CarSim 动力学", "CarSim 模型、导出变量、单位和坐标对齐。"}}},
      {"数据", ICON_FA_DATABASE,
       {{kPanelCollect, ICON_FA_DATABASE, "数据采集", "同步采集传感器数据、真值标注和车辆状态。"},
        {kPanelRecorder, ICON_FA_FILM, "录制与回放", "用 CARLA 录制器记录整个场景并回放。"},
        {kPanelView, ICON_FA_VIDEO, "实时画面", "在界面里查看主车相机画面。"}}},
  };
  return g;
}

const char* DriverName(const json& cfg) {
  if (!cfg.contains("drive")) return "-";
  const json& d = cfg["drive"];
  const bool cosim = d.value("dynamics", std::string("cosim")) == "cosim";
  const std::string v = cosim ? d.value("cosim_driver", std::string("custom")) : d.value("carla_driver", std::string("route"));
  if (v == "custom") return "我的控制算法";
  if (v == "demo") return "演示";
  if (v == "route") return "路线跟随";
  if (v == "manual") return "键盘驾驶";
  if (v == "autopilot") return "CARLA 自动驾驶";
  return "-";
}

std::string ShortName(const std::string& s) {
  const size_t k = s.find_last_of("/.");
  return k == std::string::npos ? s : s.substr(k + 1);
}

void ControlBar(const char* label, float v, float lo, float hi, const ImVec4& col, bool centered) {
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_Text, ui::Colors().text_dim);
  ImGui::TextUnformatted(label);
  ImGui::PopStyleColor();
  ImGui::SameLine(fs * 3.0f);
  ImVec2 p = ImGui::GetCursorScreenPos();
  const float w = ImGui::GetContentRegionAvail().x - fs * 3.6f;
  const float h = ImGui::GetTextLineHeight() * 0.55f;
  const float y = p.y + (ImGui::GetTextLineHeight() - h) * 0.5f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + w, y + h), ImGui::GetColorU32(ui::Colors().field), 2.0f);
  dl->AddRect(ImVec2(p.x, y), ImVec2(p.x + w, y + h), ImGui::GetColorU32(ui::Colors().card_border), 2.0f);
  const float t = std::max(0.0f, std::min(1.0f, (v - lo) / (hi - lo)));
  if (centered) {
    const float mid = p.x + w * 0.5f, x = p.x + w * t;
    dl->AddRectFilled(ImVec2(std::min(mid, x), y + 1), ImVec2(std::max(mid, x), y + h - 1), ImGui::GetColorU32(col), 1.0f);
    dl->AddLine(ImVec2(mid, y - 2), ImVec2(mid, y + h + 2), ImGui::GetColorU32(ui::Colors().text_dim));
  } else {
    dl->AddRectFilled(ImVec2(p.x + 1, y + 1), ImVec2(p.x + 1 + (w - 2) * t, y + h - 1), ImGui::GetColorU32(col), 1.0f);
  }
  // Tick marks every 25 %.
  for (int k = 1; k < 4; ++k)
    dl->AddLine(ImVec2(p.x + w * k * 0.25f, y + h), ImVec2(p.x + w * k * 0.25f, y + h + 3),
                ImGui::GetColorU32(ui::WithAlpha(ui::Colors().text_dim, 0.5f)));
  ImGui::Dummy(ImVec2(w, ImGui::GetTextLineHeight()));
  ImGui::SameLine();
  if (ui::GetFonts().mono) ImGui::PushFont(ui::GetFonts().mono);
  ImGui::Text("%+.2f", v);
  if (ui::GetFonts().mono) ImGui::PopFont();
}

bool MonitorSection(const char* icon, const char* title) { return ui::FoldHeader(icon, title, true); }

}  // namespace

// --------------------------------------------------------------------------
void App::Frame() {
  ++frame_;
  be_.Poll([this](const json& ev) { OnEvent(ev); });
  if (!be_.Connected() && plat::IsAlive(backend_proc_) && tour_dir_.empty() && frame_ % 30 == 0) ConnectBackend();
  if (tour_) TourTick();
  // Desktop launcher: connect to CARLA as soon as the backend answers.
  if (auto_connect_ && be_.Connected() && busy_.empty() && be_.PendingCount() == 0) {
    auto_connect_ = false;
    ConnectCarla();
  }
  static int last_panel = -1;
  if (panel_ != last_panel) {
    if (panel_ == kPanelCollect || panel_ == kPanelRig) RefreshDisk();
    last_panel = panel_;
  }
  UploadViewTexture();
  UpdateKeyboardDriving();
  HandleShortcuts();
  if (carla_connected_ && frame_ % 600 == 0 && !Running()) RefreshDisk();

  const ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(vp->WorkSize);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::Begin("##root", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_MenuBar |
                                      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
  ImGui::PopStyleVar(2);

  DrawMenuBar();
  DrawToolbar();
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const float status_h = ImGui::GetFrameHeight() + 2.0f;
  const float split = 5.0f * ui::Scale();
  const float total_h = ImGui::GetContentRegionAvail().y - status_h;
  if (console_h_ <= 0) console_h_ = fs * 11.0f;
  console_h_ = std::min(console_h_, total_h * 0.6f);
  const float body_h = total_h - (log_open_ ? console_h_ + split : 0.0f);
  const float total_w = ImGui::GetContentRegionAvail().x;
  if (nav_w_ <= 0) { nav_w_ = fs * 13.5f; mon_w_ = fs * 25.0f; }
  nav_w_ = std::max(fs * 9.0f, std::min(nav_w_, total_w * 0.3f));
  mon_w_ = std::max(fs * 16.0f, std::min(mon_w_, total_w * 0.45f));

  // Project tree
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.panel);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::BeginChild("nav", ImVec2(nav_w_, body_h), ImGuiChildFlags_AlwaysUseWindowPadding);
  DrawNav();
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
  ImGui::SameLine(0, 0);
  ui::Splitter("##split_nav", true, body_h, &nav_w_, fs * 9.0f, total_w * 0.3f, false);
  ImGui::SameLine(0, 0);

  // Workspace
  const float work_w = ImGui::GetContentRegionAvail().x - (monitor_open_ ? mon_w_ + split : 0.0f);
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.bg);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 1.1f, fs * 0.7f));
  ImGui::BeginChild("panel", ImVec2(work_w, body_h), ImGuiChildFlags_AlwaysUseWindowPadding);
  for (const auto& g : Nav())
    for (const auto& it : g.items)
      if (it.panel == panel_) ui::PageHeader(g.caption, it.icon, it.name, it.desc);
  switch (panel_) {
    case kPanelConnect: DrawPanelConnect(); break;
    case kPanelWorld: DrawPanelWorld(); break;
    case kPanelTraffic: DrawPanelTraffic(); break;
    case kPanelActors: DrawPanelActors(); break;
    case kPanelVehicle: DrawPanelVehicle(); break;
    case kPanelRig: DrawPanelRig(); break;
    case kPanelDrive: DrawPanelDrive(); break;
    case kPanelCoSim: DrawPanelCoSim(); break;
    case kPanelCollect: DrawPanelCollect(); break;
    case kPanelRecorder: DrawPanelRecorder(); break;
    case kPanelView: DrawPanelView(); break;
    default: break;
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();

  // Monitor
  if (monitor_open_) {
    ImGui::SameLine(0, 0);
    ui::Splitter("##split_mon", true, body_h, &mon_w_, fs * 16.0f, total_w * 0.45f, true);
    ImGui::SameLine(0, 0);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, p.panel);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::BeginChild("monitor", ImVec2(0, body_h), ImGuiChildFlags_AlwaysUseWindowPadding);
    ui::PanelTitle(ICON_FA_GAUGE_HIGH, "监视器");
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.7f, fs * 0.4f));
    ImGui::BeginChild("monitor_body", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding);
    DrawMonitor();
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
  }

  if (log_open_) {
    ui::Splitter("##split_log", false, total_w, &console_h_, fs * 5.0f, total_h * 0.6f, true);
    DrawLogDrawer();
  }
  DrawStatusBar();
  DrawAbout();
  ImGui::End();
}

// --------------------------------------------------------------------------
void App::HandleShortcuts() {
  const ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput) return;
  const bool can_run = !Running() && carla_connected_ && busy_.empty();
  if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) {
    if (io.KeyShift) {
      if (Running()) RunCommand("cosim_stop");
    } else if (run_state_ == "paused") {
      RunCommand("cosim_resume");
    } else if (can_run) {
      StartRun();
    }
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F6, false) && run_state_ == "running") RunCommand("cosim_pause");
  if (ImGui::IsKeyPressed(ImGuiKey_F10, false) && run_state_ == "paused") RunCommand("cosim_step");
  if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
    SaveConfig(cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_);
  if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_L, false)) log_open_ = !log_open_;
}

// --------------------------------------------------------------------------
void App::DrawMenuBar() {
  if (!ImGui::BeginMenuBar()) return;
  const std::string cfg = cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_;
  if (ImGui::BeginMenu("文件")) {
    if (ImGui::MenuItem(ICON_FA_FLOPPY_DISK "  保存配置", "Ctrl+S")) SaveConfig(cfg);
    if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "  配置文件 ...")) panel_ = kPanelCoSim;
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_FA_POWER_OFF "  退出")) quit_ = true;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("仿真")) {
    const bool can_run = !Running() && carla_connected_ && busy_.empty();
    if (ImGui::MenuItem(ICON_FA_PLAY "  运行", "F5", false, can_run)) StartRun();
    if (ImGui::MenuItem(ICON_FA_PAUSE "  暂停", "F6", false, run_state_ == "running")) RunCommand("cosim_pause");
    if (ImGui::MenuItem(ICON_FA_PLAY "  继续", "F5", false, run_state_ == "paused")) RunCommand("cosim_resume");
    if (ImGui::MenuItem(ICON_FA_FORWARD_STEP "  单步", "F10", false, run_state_ == "paused")) RunCommand("cosim_step");
    if (ImGui::MenuItem(ICON_FA_STOP "  停止", "Shift+F5", false, Running())) RunCommand("cosim_stop");
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_FA_PLUG "  连接 CARLA", nullptr, false, be_.Connected() && busy_.empty())) ConnectCarla();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("视图")) {
    ImGui::MenuItem(ICON_FA_GAUGE_HIGH "  监视器", nullptr, &monitor_open_);
    ImGui::MenuItem(ICON_FA_TERMINAL "  输出", "Ctrl+L", &log_open_);
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_FA_MOON "  深色主题", nullptr, dark_) && !dark_) { dark_ = true; theme_changed_ = true; SavePrefs(); }
    if (ImGui::MenuItem(ICON_FA_SUN "  浅色主题", nullptr, !dark_) && dark_) { dark_ = false; theme_changed_ = true; SavePrefs(); }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("转到")) {
    for (const auto& g : Nav())
      for (const auto& it : g.items)
        if (ImGui::MenuItem(Fmt("%s  %s", it.icon, it.name).c_str(), nullptr, panel_ == it.panel,
                            it.panel == kPanelConnect || carla_connected_))
          panel_ = it.panel;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("帮助")) {
    if (ImGui::MenuItem(ICON_FA_CIRCLE_INFO "  关于")) about_open_ = true;
    ImGui::EndMenu();
  }
  ImGui::EndMenuBar();
}

void App::DrawAbout() {
  if (about_open_) {
    ImGui::OpenPopup("关于 CARLA CoSim Studio");
    about_open_ = false;
  }
  const ImVec2 c = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  if (ImGui::BeginPopupModal("关于 CARLA CoSim Studio", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    const ui::Palette& p = ui::Colors();
    if (ui::GetFonts().title) ImGui::PushFont(ui::GetFonts().title);
    ImGui::TextColored(p.accent, ICON_FA_CAR);
    ImGui::SameLine();
    ImGui::TextUnformatted("CARLA CoSim Studio");
    if (ui::GetFonts().title) ImGui::PopFont();
    ImGui::TextColored(p.text_dim, "CARLA 0.9.16 场景仿真 + CarSim 车辆动力学联合仿真平台");
    ImGui::Separator();
    if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_SizingFixedFit)) {
      static const char* kKeys[][2] = {{"F5", "运行 / 继续"}, {"F6", "暂停"}, {"F10", "单步（暂停时）"},
                                       {"Shift+F5", "停止"}, {"Ctrl+S", "保存配置"}, {"Ctrl+L", "显示 / 隐藏输出"},
                                       {"W A S D", "键盘驾驶"}};
      for (const auto& k : kKeys) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextColored(p.accent, "%s", k[0]);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(k[1]);
      }
      ImGui::EndTable();
    }
    ImGui::Separator();
    if (ui::Button("", "关闭", ui::Kind::Primary, ImVec2(ImGui::GetFontSize() * 6, 0))) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
}

// --------------------------------------------------------------------------
void App::DrawToolbar() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const float h = fs * 2.55f;
  const float bh = fs * 1.9f;
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.chrome);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.5f, (h - bh) * 0.5f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2, 0));
  ImGui::BeginChild("toolbar", ImVec2(0, h), ImGuiChildFlags_AlwaysUseWindowPadding,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  {
    const ImVec2 wp = ImGui::GetWindowPos();
    ImGui::GetWindowDrawList()->AddLine(ImVec2(wp.x, wp.y + h - 1), ImVec2(wp.x + ImGui::GetWindowWidth(), wp.y + h - 1),
                                        ImGui::GetColorU32(p.card_border));
  }

  const bool active = Running();
  ImGui::BeginDisabled(active || !carla_connected_ || !busy_.empty());
  if (ui::ToolButton(ICON_FA_PLAY, "运行", p.success, "开始仿真 (F5)", bh)) StartRun();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (run_state_ == "paused") {
    if (ui::ToolButton(ICON_FA_PLAY, "继续", p.success, "继续 (F5)", bh)) RunCommand("cosim_resume");
  } else {
    ImGui::BeginDisabled(run_state_ != "running");
    if (ui::ToolButton(ICON_FA_PAUSE, "暂停", p.warning, "暂停 (F6)", bh)) RunCommand("cosim_pause");
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(run_state_ != "paused");
  if (ui::ToolButton(ICON_FA_FORWARD_STEP, "单步", p.accent, "暂停时前进一帧 (F10)", bh)) RunCommand("cosim_step");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!active);
  if (ui::ToolButton(ICON_FA_STOP, "停止", p.danger, "停止仿真，主车停住 (Shift+F5)", bh)) RunCommand("cosim_stop");
  ImGui::EndDisabled();
  ui::ToolSeparator(bh);
  ImGui::BeginDisabled(!be_.Connected() || !busy_.empty() || active);
  if (ui::ToolButton(ICON_FA_PLUG, carla_connected_ ? "重新连接" : "连接", p.accent, "连接 CARLA 服务器", bh)) ConnectCarla();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ui::ToolButton(ICON_FA_FLOPPY_DISK, "保存", p.text_dim, "保存当前配置 (Ctrl+S)", bh))
    SaveConfig(cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_);
  ui::ToolSeparator(bh);
  if (ui::ToolButton(ICON_FA_TERMINAL, "输出", log_open_ ? p.accent : p.text_dim, "显示 / 隐藏输出窗口 (Ctrl+L)", bh))
    log_open_ = !log_open_;
  ImGui::SameLine();
  if (ui::ToolButton(ICON_FA_GAUGE_HIGH, "监视器", monitor_open_ ? p.accent : p.text_dim, "显示 / 隐藏监视器", bh))
    monitor_open_ = !monitor_open_;

  // Right: simulation clock, like an instrument panel.
  const bool have = !last_tel_.empty() && Running();
  ImFont* mono = ui::GetFonts().mono ? ui::GetFonts().mono : ImGui::GetFont();
  const float box_w = fs * 29.5f;
  const float right = ImGui::GetWindowContentRegionMax().x;
  ImGui::SameLine(std::max(ImGui::GetCursorPosX() + fs, right - box_w));
  const ImVec2 bp = ImGui::GetCursorScreenPos();
  const float bw = ImGui::GetWindowPos().x + right - bp.x;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(bp, ImVec2(bp.x + bw, bp.y + bh), ImGui::GetColorU32(p.field), 2.0f);
  dl->AddRect(bp, ImVec2(bp.x + bw, bp.y + bh), ImGui::GetColorU32(p.card_border), 2.0f);
  const ImVec4 st_col = run_state_ == "running" ? p.success : run_state_ == "paused" ? p.warning
                      : run_state_ == "error" ? p.danger : p.text_dim;
  const char* st = run_state_ == "running" ? "运行中" : run_state_ == "paused" ? "已暂停"
                 : run_state_ == "finished" ? "已完成" : run_state_ == "error" ? "出错" : "空闲";
  const float cy = bp.y + bh * 0.5f;
  const bool blink = run_state_ != "running" || (frame_ / 30) % 2 == 0;
  dl->AddCircleFilled(ImVec2(bp.x + fs * 0.8f, cy), fs * 0.24f, ImGui::GetColorU32(ui::WithAlpha(st_col, blink ? 1.0f : 0.35f)));
  dl->AddText(ImVec2(bp.x + fs * 1.3f, cy - fs * 0.5f), ImGui::GetColorU32(st_col), st);
  // Fixed-width fields: caption on the left, value right-aligned, unit after it.
  float x = bp.x + fs * 5.2f;
  auto field = [&](const char* label, const std::string& value, const char* unit, float w) {
    const float ls = fs * 0.82f;
    dl->AddLine(ImVec2(x - fs * 0.5f, bp.y + bh * 0.22f), ImVec2(x - fs * 0.5f, bp.y + bh * 0.78f), ImGui::GetColorU32(p.card_border));
    dl->AddText(ImGui::GetFont(), ls, ImVec2(x, cy - ls * 0.5f), ImGui::GetColorU32(p.text_dim), label);
    const float uw = ImGui::GetFont()->CalcTextSizeA(ls, 1e9f, 0, unit).x;
    const float vw = mono->CalcTextSizeA(mono->FontSize, 1e9f, 0, value.c_str()).x;
    const float right_x = x + w - fs * 0.9f;
    dl->AddText(mono, mono->FontSize, ImVec2(right_x - uw - fs * 0.3f - vw, cy - mono->FontSize * 0.5f),
                ImGui::GetColorU32(have ? p.text : p.text_dim), value.c_str());
    dl->AddText(ImGui::GetFont(), ls, ImVec2(right_x - uw, cy - ls * 0.5f), ImGui::GetColorU32(p.text_dim), unit);
    x += w;
  };
  const int nf = last_tel_.value("n_frames", 0);
  const double dur = nf > 0 ? nf * cfg_["sync"].value("frame_dt", 0.02) : 0.0;
  field("时间", have ? Fmt("%.2f", last_tel_.value("t", 0.0)) : std::string("-"), "s", fs * 8.0f);
  field("车速", have ? Fmt("%.1f", last_tel_.value("speed_kmh", 0.0)) : std::string("-"), "km/h", fs * 9.0f);
  field("实时", have ? Fmt("%.2f", last_tel_.value("rt_factor", 0.0)) : std::string("-"), "x", fs * 6.8f);
  if (have && dur > 0) {
    const float frac = static_cast<float>(std::min(1.0, last_tel_.value("t", 0.0) / dur));
    dl->AddRectFilled(ImVec2(bp.x + 1, bp.y + bh - 3), ImVec2(bp.x + 1 + (bw - 2) * frac, bp.y + bh - 1),
                      ImGui::GetColorU32(p.accent));
  }
  ImGui::Dummy(ImVec2(bw, bh));

  ImGui::EndChild();
  ImGui::PopStyleVar(2);
  ImGui::PopStyleColor();
}

// --------------------------------------------------------------------------
void App::DrawNav() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ui::PanelTitle(ICON_FA_FOLDER_TREE, "工程");
  const float row_h = fs * 1.75f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const bool cosim = cfg_.contains("drive") && cfg_["drive"].value("dynamics", std::string("cosim")) == "cosim";
  auto badge = [&](int panel) -> std::string {
    switch (panel) {
      case kPanelConnect: return carla_connected_ ? "已连接" : "未连接";
      case kPanelWorld: return carla_connected_ ? ShortName(world_.value("map", std::string())) : "";
      case kPanelVehicle: return cfg_.contains("carla") ? ShortName(cfg_["carla"].value("vehicle", std::string())) : "";
      case kPanelRig: return Fmt("%d", static_cast<int>(RigSensors().size()));
      case kPanelDrive: return cosim ? "CarSim" : "CARLA";
      case kPanelCollect: return cfg_.contains("collect") && cfg_["collect"].value("enabled", false) ? "开" : "关";
      case kPanelRecorder: return recording_ ? "REC" : "";
      case kPanelView: return view_on_ ? "开" : "";
      default: return "";
    }
  };
  int gi = 0;
  for (const auto& g : Nav()) {
    bool& collapsed = nav_collapsed_[gi++];
    ImGui::PushID(g.caption);
    // Group row
    ImVec2 pos = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    if (ImGui::InvisibleButton("##grp", ImVec2(w, row_h))) collapsed = !collapsed;
    if (ImGui::IsItemHovered()) dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + row_h), ImGui::GetColorU32(ui::WithAlpha(p.text, 0.04f)));
    const float ty = pos.y + (row_h - fs) * 0.5f;
    dl->AddText(ImGui::GetFont(), fs * 0.75f, ImVec2(pos.x + fs * 0.6f, ty + fs * 0.12f), ImGui::GetColorU32(p.text_dim),
                collapsed ? ICON_FA_CHEVRON_RIGHT : ICON_FA_CHEVRON_DOWN);
    dl->AddText(ImVec2(pos.x + fs * 1.5f, ty), ImGui::GetColorU32(p.text_dim), g.icon);
    ImFont* bold = ui::GetFonts().bold ? ui::GetFonts().bold : ImGui::GetFont();
    dl->AddText(bold, fs, ImVec2(pos.x + fs * 3.0f, ty), ImGui::GetColorU32(p.text), g.caption);
    if (!collapsed) {
      for (const auto& it : g.items) {
        const bool disabled = it.panel != kPanelConnect && !carla_connected_;
        const bool sel = panel_ == it.panel;
        ImGui::PushID(it.panel);
        pos = ImGui::GetCursorScreenPos();
        ImGui::BeginDisabled(disabled);
        if (ImGui::InvisibleButton("##nav", ImVec2(w, row_h))) panel_ = it.panel;
        const bool hov = ImGui::IsItemHovered();
        ImGui::EndDisabled();
        if (sel) {
          dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + row_h), ImGui::GetColorU32(ui::WithAlpha(p.accent, ui::IsDark() ? 0.28f : 0.16f)));
          dl->AddRectFilled(pos, ImVec2(pos.x + 2.5f, pos.y + row_h), ImGui::GetColorU32(p.accent));
        } else if (hov) {
          dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + row_h), ImGui::GetColorU32(ui::WithAlpha(p.text, 0.05f)));
        }
        // Tree guide line
        dl->AddLine(ImVec2(pos.x + fs * 1.95f, pos.y), ImVec2(pos.x + fs * 1.95f, pos.y + row_h),
                    ImGui::GetColorU32(ui::WithAlpha(p.text_dim, 0.25f)));
        const float iy = pos.y + (row_h - fs) * 0.5f;
        dl->AddText(ImVec2(pos.x + fs * 2.6f, iy), ImGui::GetColorU32(disabled ? ui::WithAlpha(p.text_dim, 0.5f) : (sel ? p.accent : p.text_dim)), it.icon);
        dl->AddText(ImVec2(pos.x + fs * 4.1f, iy), ImGui::GetColorU32(disabled ? ui::WithAlpha(p.text_dim, 0.6f) : p.text), it.name);
        const std::string b = badge(it.panel);
        if (!b.empty() && !disabled) {
          const float bs = fs * 0.8f;
          const float bw = ImGui::GetFont()->CalcTextSizeA(bs, 1e9f, 0, b.c_str()).x;
          const float nx = pos.x + fs * 4.1f + ImGui::CalcTextSize(it.name).x + fs * 0.6f;
          const float bx = std::max(nx, pos.x + w - bw - fs * 0.7f);
          if (bx + bw < pos.x + w - 2) {
            const ImVec4 bc = it.panel == kPanelConnect ? (carla_connected_ ? p.success : p.danger)
                            : it.panel == kPanelRecorder ? p.danger : p.text_dim;
            dl->AddText(ImGui::GetFont(), bs, ImVec2(bx, pos.y + (row_h - bs) * 0.5f), ImGui::GetColorU32(bc), b.c_str());
          }
        }
        ImGui::PopID();
      }
    }
    ImGui::PopID();
  }
}

// --------------------------------------------------------------------------
void App::DrawMonitor() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();

  if (view_on_ && view_tex_ && MonitorSection(ICON_FA_VIDEO, "实时画面")) {
    DrawViewImage(ImGui::GetContentRegionAvail().x, fs * 12.0f);
    ImGui::Dummy(ImVec2(0, 2));
  }

  const bool have = !last_tel_.empty();
  if (MonitorSection(ICON_FA_GAUGE_HIGH, "运行状态")) {
    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const float tile_w = (ImGui::GetContentRegionAvail().x - gap) / 2.0f;
    ui::KpiTile("车速", have ? Fmt("%.1f", last_tel_.value("speed_kmh", 0.0)).c_str() : "-", "km/h", tile_w);
    ImGui::SameLine();
    ui::KpiTile("仿真时间", have ? Fmt("%.2f", last_tel_.value("t", 0.0)).c_str() : "-", "s", tile_w);
    ui::KpiTile("实时倍率", have ? Fmt("%.2f", last_tel_.value("rt_factor", 0.0)).c_str() : "-", "x", tile_w);
    ImGui::SameLine();
    ui::KpiTile("帧", have ? Fmt("%d", last_tel_.value("frame", 0)).c_str() : "-", "", tile_w);
    if (have && last_tel_.value("at_red_light", false)) ImGui::TextColored(p.danger, ICON_FA_TRAFFIC_LIGHT "  正在等红灯");
    ImGui::Dummy(ImVec2(0, 2));
  }

  if (MonitorSection(ICON_FA_SLIDERS, "控制输入")) {
    const json a = have ? last_tel_["action"] : json::array();
    const bool cosim = last_tel_.value("dynamics", std::string()) == "CarSim";
    const float thr = a.size() > 0 ? a[0].get<float>() : 0.0f, brk = a.size() > 1 ? a[1].get<float>() : 0.0f;
    // CarSim action[2] is the steering wheel angle (deg, + left); CARLA's is steer (-1..1, + right).
    const float sw = a.size() > 2 ? a[2].get<float>() : 0.0f;
    const float steer = cosim ? -sw / std::max(1.0f, cfg_["sync"].value("steering_wheel_max_deg", 540.0f)) : sw;
    ControlBar("油门", thr, 0, 1, p.success, false);
    ControlBar("制动", std::min(1.0f, brk), 0, 1, p.danger, false);
    ControlBar("转向", steer, -1, 1, p.accent, true);
    ImGui::Dummy(ImVec2(0, 2));
  }

  if (MonitorSection(ICON_FA_LOCATION_CROSSHAIRS, "位姿与车轮")) {
    if (have && ImGui::BeginTable("pose", 4, ImGuiTableFlags_SizingStretchSame)) {
      const json& loc = last_tel_["location"];
      const json& rot = last_tel_["rotation"];
      const char* names[] = {"X m", "Y m", "Z m", "航向 °", "俯仰 °", "侧倾 °"};
      const double vals[] = {loc[0].get<double>(), loc[1].get<double>(), loc[2].get<double>(),
                             rot[1].get<double>(), rot[0].get<double>(), rot[2].get<double>()};
      for (int i = 0; i < 6; ++i) {
        if (i % 2 == 0) ImGui::TableNextRow();
        ImGui::TableSetColumnIndex((i % 2) * 2);
        ImGui::TextColored(p.text_dim, "%s", names[i]);
        ImGui::TableSetColumnIndex((i % 2) * 2 + 1);
        if (ui::GetFonts().mono) ImGui::PushFont(ui::GetFonts().mono);
        ImGui::Text("%9.2f", vals[i]);
        if (ui::GetFonts().mono) ImGui::PopFont();
      }
      ImGui::EndTable();
    }
    if (ImGui::BeginTable("wheels", 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("车轮");
      ImGui::TableSetupColumn("转向 °");
      ImGui::TableSetupColumn("转角 °");
      ImGui::TableSetupColumn("悬架 mm");
      ImGui::TableHeadersRow();
      static const char* kW[] = {"左前", "右前", "左后", "右后"};
      const json empty = json::array();
      const json& st = have ? last_tel_["wheel_steer"] : empty;
      const json& ro = have ? last_tel_["wheel_rotation"] : empty;
      const json& su = have ? last_tel_["wheel_suspension_mm"] : empty;
      for (size_t i = 0; i < 4; ++i) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextColored(p.text_dim, "%s", kW[i]);
        if (ui::GetFonts().mono) ImGui::PushFont(ui::GetFonts().mono);
        ImGui::TableSetColumnIndex(1); if (i < st.size()) ImGui::Text("%+7.2f", st[i].get<double>()); else ImGui::TextDisabled("   --");
        ImGui::TableSetColumnIndex(2); if (i < ro.size()) ImGui::Text("%7.0f", ro[i].get<double>()); else ImGui::TextDisabled("   --");
        ImGui::TableSetColumnIndex(3); if (i < su.size()) ImGui::Text("%+7.1f", su[i].get<double>()); else ImGui::TextDisabled("   --");
        if (ui::GetFonts().mono) ImGui::PopFont();
      }
      ImGui::EndTable();
    }
    ImGui::Dummy(ImVec2(0, 2));
  }

  if (collect_stats_.contains("frames") && MonitorSection(ICON_FA_DATABASE, "采集进度")) {
    ImGui::Text("%d 帧   %.1f MB   %.1f MB/s", collect_stats_.value("frames", 0),
                collect_stats_.value("bytes", 0.0) / 1e6, collect_stats_.value("mb_per_s", 0.0));
    const int maxf = cfg_["collect"].value("max_frames", 0);
    if (maxf > 0)
      ImGui::ProgressBar(std::min(1.0f, collect_stats_.value("frames", 0) / static_cast<float>(maxf)), ImVec2(-1, 0));
    if (collect_stats_.value("done", false))
      ImGui::TextColored(p.success, ICON_FA_CIRCLE_CHECK "  %s", collect_stats_.value("reason", std::string()).c_str());
    ImGui::Dummy(ImVec2(0, 2));
  }

  if (!MonitorSection(ICON_FA_CHART_LINE, "曲线")) return;
  const int n = static_cast<int>(h_t_.size());
  if (n < 2) {
    ImGui::TextColored(p.text_dim, "运行后显示车速、前轮转角、悬架行程、油门 / 制动曲线");
    return;
  }
  // Y range from the data, but never narrower than a sensible span so that
  // near-constant signals do not get magnified into noise.
  auto fit = [](const std::vector<float>* series, int count, float min_span, bool from_zero) {
    float lo = 1e30f, hi = -1e30f;
    for (int k = 0; k < count; ++k)
      for (float v : series[k]) { lo = std::min(lo, v); hi = std::max(hi, v); }
    if (from_zero) lo = std::min(lo, 0.0f);
    const float mid = 0.5f * (lo + hi), half = std::max(0.5f * (hi - lo) * 1.1f, 0.5f * min_span);
    ImPlot::SetupAxisLimits(ImAxis_Y1, from_zero ? std::min(lo, mid - half) : mid - half, mid + half, ImPlotCond_Always);
  };
  const float ph = fs * 7.0f;
  ImPlotFlags pf = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect;
  ImPlotSpec line;
  line.LineWeight = 1.5f;
  ImPlotAxisFlags ax = ImPlotAxisFlags_AutoFit;
  if (ImPlot::BeginPlot("车速 (km/h)", ImVec2(-1, ph), pf | ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    fit(&h_speed_, 1, 10.0f, true);
    ImPlot::PlotLine("车速", h_t_.data(), h_speed_.data(), n, line);
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("前轮转向角 (°)", ImVec2(-1, ph), pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    const std::vector<float> steer[2] = {h_steer_fl_, h_steer_fr_};
    fit(steer, 2, 4.0f, false);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    ImPlot::PlotLine("左前", h_t_.data(), h_steer_fl_.data(), n, line);
    ImPlot::PlotLine("右前", h_t_.data(), h_steer_fr_.data(), n, line);
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("悬架行程 (mm)", ImVec2(-1, ph), pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    fit(h_susp_, 4, 10.0f, false);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    static const char* kW[] = {"左前", "右前", "左后", "右后"};
    for (int i = 0; i < 4; ++i) ImPlot::PlotLine(kW[i], h_t_.data(), h_susp_[i].data(), n, line);
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("油门 / 制动", ImVec2(-1, ph), pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -0.05, 1.05, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    ImPlot::PlotLine("油门", h_t_.data(), h_thr_.data(), n, line);
    ImPlot::PlotLine("制动", h_t_.data(), h_brk_.data(), n, line);
    ImPlot::EndPlot();
  }
}

// --------------------------------------------------------------------------
void App::DrawStatusBar() {
  const ui::Palette& p = ui::Colors();
  const bool running = run_state_ == "running";
  // Like an IDE in debug mode: the status bar turns accent-coloured while a run is active.
  const ImVec4 bg = running ? ui::WithAlpha(p.accent, 0.9f) : p.chrome;
  const ImVec4 dim = running ? ImVec4(1, 1, 1, 0.85f) : p.text_dim;
  ImGui::PushStyleColor(ImGuiCol_ChildBg, bg);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetFontSize() * 0.6f, 1));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 0));
  ImGui::BeginChild("status", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollbar);
  {
    const ImVec2 wp = ImGui::GetWindowPos();
    ImGui::GetWindowDrawList()->AddLine(wp, ImVec2(wp.x + ImGui::GetWindowWidth(), wp.y), ImGui::GetColorU32(p.card_border));
  }
  ImGui::AlignTextToFramePadding();
  auto sep = [&]() {
    ImGui::SameLine(0, 10);
    const ImVec2 q = ImGui::GetCursorScreenPos();
    const float h = ImGui::GetFrameHeight();
    ImGui::GetWindowDrawList()->AddLine(ImVec2(q.x, q.y + h * 0.25f), ImVec2(q.x, q.y + h * 0.75f),
                                        ImGui::GetColorU32(ui::WithAlpha(dim, 0.35f)));
    ImGui::Dummy(ImVec2(1, h));
    ImGui::SameLine(0, 10);
  };
  const bool be = be_.Connected();
  ui::StatusDot(be ? p.success : (plat::IsAlive(backend_proc_) ? p.warning : p.danger));
  ImGui::SameLine(0, 2);
  ImGui::TextColored(dim, "后端 %s", be ? "已连接" : (plat::IsAlive(backend_proc_) ? "启动中" : "未运行"));
  sep();
  ui::StatusDot(carla_connected_ ? p.success : p.danger);
  ImGui::SameLine(0, 2);
  if (carla_connected_)
    ImGui::TextColored(dim, "CARLA %s  ·  %s", server_info_.value("server_version", std::string()).c_str(),
                       world_.value("map", std::string()).c_str());
  else
    ImGui::TextColored(dim, "CARLA 未连接");
  if (world_.value("ego_id", 0)) {
    sep();
    ImGui::TextColored(dim, "%s 主车 #%d", ICON_FA_CAR, world_.value("ego_id", 0));
  }
  if (disk_.contains("free_gb")) {
    sep();
    const double free_gb = disk_.value("free_gb", 0.0);
    ImGui::TextColored(free_gb < 20 && !running ? p.warning : dim, "%s 磁盘剩余 %.0f GB", ICON_FA_HARD_DRIVE, free_gb);
  }
  if (!busy_.empty()) {
    sep();
    const char* spin = "|/-\\";
    ImGui::TextColored(running ? dim : p.warning, "%c %s", spin[(frame_ / 8) % 4], busy_.c_str());
  }
  const float right = ImGui::GetWindowContentRegionMax().x;
  ImGui::SameLine(right - ImGui::GetFontSize() * 9.0f);
  std::string label = log_errors_ ? Fmt(ICON_FA_TRIANGLE_EXCLAMATION " %d 个错误", log_errors_) : std::string(ICON_FA_TERMINAL " 输出");
  ImGui::PushStyleColor(ImGuiCol_Text, log_errors_ ? (running ? ImVec4(1, 1, 1, 1) : p.danger) : dim);
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
  ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
  if (ImGui::SmallButton(label.c_str())) {
    log_open_ = !log_open_;
    log_errors_ = 0;
  }
  ImGui::PopStyleColor(3);
  ImGui::SameLine();
  ImGui::TextColored(dim, "%.0f fps", ImGui::GetIO().Framerate);
  ImGui::EndChild();
  ImGui::PopStyleVar(2);
  ImGui::PopStyleColor();
}

void App::DrawLogDrawer() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.panel);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::BeginChild("logdrawer", ImVec2(0, console_h_), ImGuiChildFlags_AlwaysUseWindowPadding);
  ui::PanelTitle(ICON_FA_TERMINAL, "输出");
  // Filter + actions on the title bar, right-aligned.
  int counts[3] = {0, 0, 0};
  for (const auto& l : log_) ++counts[l.level == "error" ? 2 : l.level == "warn" ? 1 : 0];
  const float bar_h = fs * 1.9f;
  ImGui::SetCursorPos(ImVec2(ImGui::GetWindowWidth() - fs * 20.5f, (bar_h - ImGui::GetFrameHeight()) * 0.5f));
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 1));
  const char* names[] = {"全部", "警告", "错误"};
  for (int i = 0; i < 3; ++i) {
    if (i) ImGui::SameLine(0, 2);
    const bool on = log_filter_ == i;
    ImGui::PushStyleColor(ImGuiCol_Button, on ? ui::WithAlpha(p.accent, 0.3f) : ImVec4(0, 0, 0, 0));
    const std::string lbl = i == 0 ? Fmt("%s %d", names[i], static_cast<int>(log_.size()))
                                   : Fmt("%s %d", names[i], counts[i]);
    if (ImGui::SmallButton(Fmt("%s##lf%d", lbl.c_str(), i).c_str())) log_filter_ = i;
    ImGui::PopStyleColor();
  }
  ImGui::SameLine(0, fs);
  if (ImGui::SmallButton(ICON_FA_TRASH " 清空")) { log_.clear(); log_errors_ = 0; }
  ImGui::SameLine(0, 2);
  if (ImGui::SmallButton(ICON_FA_XMARK)) log_open_ = false;
  ImGui::PopStyleVar();
  ImGui::SetCursorPos(ImVec2(0, bar_h));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.6f, fs * 0.25f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 2));
  ImGui::BeginChild("logscroll", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding);
  ImFont* mono = ui::GetFonts().mono;
  for (const auto& l : log_) {
    const int lv = l.level == "error" ? 2 : l.level == "warn" ? 1 : 0;
    if (log_filter_ == 1 && lv < 1) continue;
    if (log_filter_ == 2 && lv < 2) continue;
    const ImVec4 c = lv == 2 ? p.danger : lv == 1 ? p.warning : p.text;
    if (mono) ImGui::PushFont(mono);
    ImGui::TextColored(p.text_dim, "%s", l.time.c_str());
    if (mono) ImGui::PopFont();
    ImGui::SameLine(fs * 5.2f);
    ImGui::TextColored(lv ? c : p.text_dim, "%s", lv == 2 ? ICON_FA_CIRCLE_XMARK : lv == 1 ? ICON_FA_TRIANGLE_EXCLAMATION : ICON_FA_CIRCLE_INFO);
    ImGui::SameLine(fs * 6.6f);
    ImGui::TextColored(c, "%s", l.text.c_str());
  }
  if (log_scroll_) {
    ImGui::SetScrollHereY(1.0f);
    log_scroll_ = false;
  }
  ImGui::EndChild();
  ImGui::PopStyleVar(2);
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}
