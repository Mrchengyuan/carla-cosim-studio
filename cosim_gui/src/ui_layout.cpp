// Window shell in the style of vehicle-simulation tools (CarMaker, VTD):
// menu bar and toolbar on top, project tree on the left, the live 3D view in
// the centre with instrument / minimap overlays, a tabbed dock below it
// (plots, vehicle state, output), the properties of the selected page on the
// right, status bar at the bottom. Every divider can be dragged.
#include <algorithm>
#include <cmath>

#include "app.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "ui_kit.h"

using appui::Fmt;

namespace {

constexpr float kPi = 3.14159265f;

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
        {kPanelView, ICON_FA_VIDEO, "实时画面", "中间视口的相机设置：视角、套件相机、分辨率。"},
        {kPanelDataset, ICON_FA_FOLDER_OPEN, "数据浏览", "逐帧查看已采集的数据和真值框，导出为 KITTI / nuScenes，删除不需要的数据集。"}}},
  };
  return g;
}

std::string ShortName(const std::string& s) {
  const size_t k = s.find_last_of("/.");
  return k == std::string::npos ? s : s.substr(k + 1);
}

// Overlay colours on top of the camera image: always light-on-dark.
const ImVec4 kHudBg(0.05f, 0.06f, 0.07f, 0.62f);
const ImVec4 kHudBorder(1, 1, 1, 0.10f);
const ImVec4 kHudText(0.92f, 0.93f, 0.95f, 1);
const ImVec4 kHudDim(0.92f, 0.93f, 0.95f, 0.55f);

void HudPanel(ImDrawList* dl, ImVec2 a, ImVec2 b) {
  dl->AddRectFilled(a, b, ImGui::GetColorU32(kHudBg), 4.0f);
  dl->AddRect(a, b, ImGui::GetColorU32(kHudBorder), 4.0f);
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
  for (int k = 1; k < 4; ++k)
    dl->AddLine(ImVec2(p.x + w * k * 0.25f, y + h), ImVec2(p.x + w * k * 0.25f, y + h + 3),
                ImGui::GetColorU32(ui::WithAlpha(ui::Colors().text_dim, 0.5f)));
  ImGui::Dummy(ImVec2(w, ImGui::GetTextLineHeight()));
  ImGui::SameLine();
  if (ui::GetFonts().mono) ImGui::PushFont(ui::GetFonts().mono);
  ImGui::Text("%+.2f", v);
  if (ui::GetFonts().mono) ImGui::PopFont();
}

// Toolbar tab that jumps to a page.
bool ToolTab(const char* label, bool active, float height) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const bool disabled = (ImGui::GetCurrentContext()->CurrentItemFlags & ImGuiItemFlags_Disabled) != 0;
  const ImVec2 ts = ImGui::CalcTextSize(label);
  const ImVec2 size(ts.x + fs * 1.4f, height);
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const bool clicked = ImGui::InvisibleButton(label, size);
  ui::RecordTarget(std::string("tab:") + label);
  const bool hov = ImGui::IsItemHovered();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  if (active) {
    dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), ImGui::GetColorU32(ui::WithAlpha(p.accent, 0.16f)), 3.0f);
    dl->AddRectFilled(ImVec2(pos.x + fs * 0.4f, pos.y + size.y - 2.5f), ImVec2(pos.x + size.x - fs * 0.4f, pos.y + size.y),
                      ImGui::GetColorU32(p.accent));
  } else if (hov && !disabled) {
    dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), ImGui::GetColorU32(ImGuiCol_ButtonHovered), 3.0f);
  }
  dl->AddText(ImVec2(pos.x + fs * 0.7f, pos.y + (height - fs) * 0.5f),
              ImGui::GetColorU32(disabled ? ui::WithAlpha(p.text_dim, 0.5f) : (active ? p.text : p.text_dim)), label);
  return clicked && !disabled;
}

}  // namespace

// --------------------------------------------------------------------------
void App::Frame() {
  ++frame_;
  be_.Poll([this](const json& ev) { OnEvent(ev); });
  if (!be_.Connected() && plat::IsAlive(backend_proc_) && tour_dir_.empty() && frame_ % 30 == 0) ConnectBackend(true);
  if (tour_) TourTick();
  // Desktop launcher: connect to CARLA as soon as the backend answers.
  if (auto_connect_ && be_.Connected() && busy_.empty() && be_.PendingCount() == 0) {
    auto_connect_ = false;
    ConnectCarla();
  }
  if (recover_connect_ && be_.Connected() && busy_.empty() && be_.PendingCount() == 0) {
    recover_connect_ = false;
    ConnectCarla(true);
  }
  // A backend that hangs (a CARLA call that never returns) or died: say so,
  // with a way out, instead of a GUI that silently stops updating.
  if (!busy_task_.empty() && ImGui::GetTime() - busy_seen_ > 3.0) busy_task_.clear();  // finished since
  if (!busy_task_.empty()) {
    const bool frame_task = busy_task_ == "仿真步进" || busy_task_ == "空闲时推进世界";
    const double limit = frame_task ? 15 : (busy_task_ == "load_map" || busy_task_ == "reload_world") ? 300
                       : busy_task_ == "vehicle_specs" ? 900 : 90;
    if (busy_carla_gone_)  // not stuck: waiting for a server that has gone, it gives up soon
      backend_problem_ = Fmt("后端卡住了：CARLA 服务器已退出，正在结束“%s”（最多约半分钟）……", busy_task_.c_str());
    else if (busy_secs_ > limit)
      backend_problem_ = Fmt("后端卡住了：“%s”已经 %.0f 秒没有完成，多半是 CARLA 的客户端库卡死了。",
                             busy_task_.c_str(), busy_secs_);
  } else if (backend_problem_.rfind("后端卡住了", 0) == 0) {
    backend_problem_.clear();  // it went on after all
  }
  if (backend_proc_.valid() && !be_.Connected() && !plat::IsAlive(backend_proc_) && backend_problem_.empty()) {
    // (Not probing CARLA's port here: CARLA 0.9.16 can crash on connections
    // that close right away. With the original carla package the backend
    // dies when CARLA does, hence the hint.)
    backend_problem_ = "后端进程意外退出（" + plat::ExitDescription(backend_proc_) +
                       "）。如果 CARLA 也退出了，先重新启动 CARLA。出错记录在桥接目录的 backend.log，重启后保存为 backend.prev.log。";
  }
  // The centre viewport shows the ego camera as soon as there is an ego.
  const int ego = world_.value("ego_id", 0);
  if (view_auto_ && carla_connected_ && ego && ego != view_auto_ego_ && !view_on_ && busy_.empty() && tour_dir_.empty()) {
    view_auto_ego_ = ego;
    StartView();
  }
  static int last_panel = -1;
  if (panel_ != last_panel) {
    if (panel_ == kPanelCollect || panel_ == kPanelRig) RefreshDisk();
    last_panel = panel_;
  }
  DatasetTick();
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
  const float gap_y = ImGui::GetStyle().ItemSpacing.y;
  DrawToolbar();
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const float split = 5.0f * ui::Scale();
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
  ImGui::SetCursorPosY(ImGui::GetCursorPosY() - gap_y);
  const float status_h = ImGui::GetFrameHeight() + 2.0f;
  const float total_h = ImGui::GetContentRegionAvail().y - status_h;
  const float total_w = ImGui::GetContentRegionAvail().x;
  if (nav_w_ <= 0) { nav_w_ = fs * 13.0f; mon_w_ = fs * 34.0f; console_h_ = fs * 15.0f; }
  nav_w_ = std::max(fs * 9.0f, std::min(nav_w_, total_w * 0.25f));
  mon_w_ = std::max(fs * 24.0f, std::min(mon_w_, total_w * 0.5f));
  console_h_ = std::max(fs * 6.0f, std::min(console_h_, total_h * 0.7f));

  // Project tree
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.panel);
  ImGui::BeginChild("nav", ImVec2(nav_w_, total_h));
  DrawNav();
  ImGui::EndChild();
  ImGui::PopStyleColor();
  ImGui::SameLine();
  ui::Splitter("##split_nav", true, total_h, &nav_w_, fs * 9.0f, total_w * 0.25f, false);
  ImGui::SameLine();

  // Centre: viewport + dock
  const float center_w = ImGui::GetContentRegionAvail().x - (monitor_open_ ? mon_w_ + split : 0.0f);
  ImGui::BeginChild("center", ImVec2(center_w, total_h));
  const float view_h = total_h - (log_open_ ? console_h_ + split : 0.0f);
  DrawViewport(center_w, view_h);
  if (log_open_) {
    ui::Splitter("##split_dock", false, center_w, &console_h_, fs * 6.0f, total_h * 0.7f, true);
    DrawDock(center_w, console_h_);
  }
  ImGui::EndChild();

  // Properties of the selected page
  if (monitor_open_) {
    ImGui::SameLine();
    ui::Splitter("##split_prop", true, total_h, &mon_w_, fs * 24.0f, total_w * 0.5f, true);
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_ChildBg, p.bg);
    ImGui::BeginChild("props", ImVec2(0, total_h));
    DrawProperties();
    ImGui::EndChild();
    ImGui::PopStyleColor();
  }

  DrawStatusBar();
  ImGui::PopStyleVar();
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
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 6));
  const std::string cfg = cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_;
  if (ImGui::BeginMenu("文件")) {
    if (ImGui::MenuItem(ICON_FA_FLOPPY_DISK "  保存配置", "Ctrl+S")) SaveConfig(cfg);
    if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "  配置文件 ...")) { panel_ = kPanelCoSim; monitor_open_ = true; }
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
    if (ImGui::MenuItem(ICON_FA_PLUG "  连接 CARLA", nullptr, false, be_.Connected() && busy_.empty() && !Running())) ConnectCarla();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("视图")) {
    ImGui::MenuItem(ICON_FA_SLIDERS "  属性面板", nullptr, &monitor_open_);
    ImGui::MenuItem(ICON_FA_TABLE_COLUMNS "  底部面板（曲线 / 状态 / 输出）", "Ctrl+L", &log_open_);
    if (ImGui::MenuItem(ICON_FA_VIDEO "  实时画面", nullptr, view_on_, world_.value("ego_id", 0) != 0)) {
      if (view_on_) { Call("view_stop", json::object(), nullptr); view_on_ = false; view_auto_ = false; }
      else { view_auto_ = true; StartView(); }
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_FA_MOON "  深色主题", nullptr, dark_) && !dark_) { dark_ = true; theme_changed_ = true; SavePrefs(); }
    if (ImGui::MenuItem(ICON_FA_SUN "  浅色主题", nullptr, !dark_) && dark_) { dark_ = false; theme_changed_ = true; SavePrefs(); }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("转到")) {
    for (const auto& g : Nav())
      for (const auto& it : g.items)
        if (ImGui::MenuItem(Fmt("%s  %s", it.icon, it.name).c_str(), nullptr, panel_ == it.panel,
                            it.panel == kPanelConnect || carla_connected_)) {
          panel_ = it.panel;
          monitor_open_ = true;
        }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("帮助")) {
    if (ImGui::MenuItem(ICON_FA_CIRCLE_INFO "  关于与快捷键")) about_open_ = true;
    ImGui::EndMenu();
  }
  ImGui::PopStyleVar();
  ImGui::EndMenuBar();
}

void App::DrawAbout() {
  if (about_open_) {
    ImGui::OpenPopup("关于 CARLA CoSim Studio");
    about_open_ = false;
  }
  const ImVec2 c = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16, 14));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 6));
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
                                       {"Shift+F5", "停止"}, {"Ctrl+S", "保存配置"}, {"Ctrl+L", "显示 / 隐藏底部面板"},
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
  ImGui::PopStyleVar(2);
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
  if (ui::ToolButton(ICON_FA_PLUG, carla_connected_ ? "重连" : "连接", p.accent, "连接 CARLA 服务器", bh)) ConnectCarla();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ui::ToolButton(ICON_FA_FLOPPY_DISK, "保存", p.text_dim, "保存当前配置 (Ctrl+S)", bh))
    SaveConfig(cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_);
  ui::ToolSeparator(bh);

  // Workflow tabs: jump straight to the page of each step.
  struct T { const char* label; int panel; };
  static const T kTabs[] = {{"场景", kPanelWorld}, {"车辆", kPanelVehicle}, {"传感器", kPanelRig},
                            {"驾驶", kPanelDrive}, {"CarSim", kPanelCoSim}, {"采集", kPanelCollect}};
  ImGui::BeginDisabled(!carla_connected_);
  for (size_t i = 0; i < 6; ++i) {
    if (i) ImGui::SameLine();
    if (ToolTab(kTabs[i].label, panel_ == kTabs[i].panel, bh)) {
      panel_ = kTabs[i].panel;
      monitor_open_ = true;
    }
  }
  ImGui::EndDisabled();

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
    ImVec2 pos = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    if (ImGui::InvisibleButton("##grp", ImVec2(w, row_h))) collapsed = !collapsed;
    if (ImGui::IsItemHovered()) dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + row_h), ImGui::GetColorU32(ui::WithAlpha(p.text, 0.04f)));
    const float ty = pos.y + (row_h - fs) * 0.5f;
    dl->AddText(ImGui::GetFont(), fs * 0.72f, ImVec2(pos.x + fs * 0.6f, ty + fs * 0.14f), ImGui::GetColorU32(p.text_dim),
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
        if (ImGui::InvisibleButton("##nav", ImVec2(w, row_h))) {
          panel_ = it.panel;
          monitor_open_ = true;
        }
        ui::RecordTarget(std::string("nav:") + it.name);
        const bool hov = ImGui::IsItemHovered();
        ImGui::EndDisabled();
        if (sel) {
          dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + row_h), ImGui::GetColorU32(ui::WithAlpha(p.accent, ui::IsDark() ? 0.28f : 0.16f)));
          dl->AddRectFilled(pos, ImVec2(pos.x + 2.5f, pos.y + row_h), ImGui::GetColorU32(p.accent));
        } else if (hov) {
          dl->AddRectFilled(pos, ImVec2(pos.x + w, pos.y + row_h), ImGui::GetColorU32(ui::WithAlpha(p.text, 0.05f)));
        }
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
void App::DrawProperties() {
  const float fs = ImGui::GetFontSize();
  ui::PanelTitle(ICON_FA_SLIDERS, "属性");
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.8f, fs * 0.6f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(7 * ui::Scale(), 6 * ui::Scale()));
  ImGui::BeginChild("props_body", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding);
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
    case kPanelDataset: DrawPanelDataset(); break;
    default: break;
  }
  if (props_scroll_end_) {
    ImGui::SetScrollHereY(1.0f);
    props_scroll_end_ = false;
  }
  ImGui::EndChild();
  ImGui::PopStyleVar(2);
}

// --------------------------------------------------------------------------
// Centre viewport: ego camera filling the area, with a camera bar, the
// instrument cluster and a minimap drawn over it.
void App::DrawViewport(float w, float h) {
  if (panel_ == kPanelDataset) {  // the viewport shows the dataset being browsed
    DrawDatasetViewport(w, h);
    return;
  }
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.055f, 0.06f, 0.07f, 1));
  ImGui::BeginChild("viewport", ImVec2(w, h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o = ImGui::GetWindowPos();
  const ImVec2 e(o.x + w, o.y + h);
  const int ego = world_.value("ego_id", 0);

  // Background grid, visible when there is no image.
  const ImU32 grid = ImGui::GetColorU32(ImVec4(1, 1, 1, 0.035f));
  for (float gx = o.x; gx < e.x; gx += fs * 3) dl->AddLine(ImVec2(gx, o.y), ImVec2(gx, e.y), grid);
  for (float gy = o.y; gy < e.y; gy += fs * 3) dl->AddLine(ImVec2(o.x, gy), ImVec2(e.x, gy), grid);

  // Pane layout: the main camera pane (mo, mw x mh) plus up to three more.
  const bool multi = view_on_ && carla_connected_ && ego && view_layout_ > 0;
  const float gap = 3.0f;
  ImVec2 mo = o;
  float mw = w, mh = h;
  if (multi && view_layout_ == 1) {
    mw = std::floor(w * 0.68f);
    const float sw = w - mw - gap, sh = (h - 2 * gap) / 3.0f;
    for (int i = 1; i < 4; ++i) DrawPane(i, ImVec2(o.x + mw + gap, o.y + (i - 1) * (sh + gap)), ImVec2(sw, sh));
  } else if (multi && view_layout_ == 2) {
    mw = (w - gap) * 0.5f;
    mh = (h - gap) * 0.5f;
    DrawPane(1, ImVec2(o.x + mw + gap, o.y), ImVec2(mw, mh));
    DrawPane(2, ImVec2(o.x, o.y + mh + gap), ImVec2(mw, mh));
    DrawPane(3, ImVec2(o.x + mw + gap, o.y + mh + gap), ImVec2(mw, mh));
  }
  const ImVec2 me(mo.x + mw, mo.y + mh);
  if (multi) {
    dl->AddRectFilled(mo, me, ImGui::GetColorU32(ImVec4(0.03f, 0.035f, 0.04f, 1)));
    dl->AddRect(mo, me, ImGui::GetColorU32(ImVec4(1, 1, 1, 0.12f)));
  }

  const bool image = view_on_ && view_tex_ && view_w_ > 0 && view_h_ > 0;
  if (image) {
    const float s = std::min(mw / view_w_, mh / view_h_);
    const ImVec2 sz(view_w_ * s, view_h_ * s);
    const ImVec2 a(mo.x + (mw - sz.x) * 0.5f, mo.y + (mh - sz.y) * 0.5f);
    dl->AddImage(static_cast<ImTextureID>(static_cast<intptr_t>(view_tex_)), a, ImVec2(a.x + sz.x, a.y + sz.y));
  } else {
    // Placeholder with the next step to take.
    const char* icon = ICON_FA_VIDEO_SLASH;
    std::string msg, sub;
    int action = 0;
    if (!be_.Connected() && !plat::IsAlive(backend_proc_)) { msg = "后端没有运行"; sub = "界面靠 Python 后端和 CARLA 通信"; action = 4; }
    else if (!carla_connected_) { msg = "未连接 CARLA"; sub = "启动 CARLA 服务器后点“连接”"; action = 1; }
    else if (!ego) { msg = "还没有主车"; sub = "在“车辆与视角”里选择车型和出生点"; action = 2; }
    else if (view_on_) { msg = "正在等待画面 ..."; icon = ICON_FA_SPINNER; }
    else { msg = "实时画面已关闭"; action = 3; }
    ImFont* title = ui::GetFonts().title ? ui::GetFonts().title : ImGui::GetFont();
    const ImVec2 c(o.x + w * 0.5f, o.y + h * 0.45f);
    const float big = fs * 3.0f;
    dl->AddText(ImGui::GetFont(), big, ImVec2(c.x - big * 0.55f, c.y - big * 1.6f), ImGui::GetColorU32(kHudDim), icon);
    const ImVec2 ms = title->CalcTextSizeA(title->FontSize, 1e9f, 0, msg.c_str());
    dl->AddText(title, title->FontSize, ImVec2(c.x - ms.x * 0.5f, c.y), ImGui::GetColorU32(kHudText), msg.c_str());
    if (!sub.empty()) {
      const ImVec2 ss = ImGui::CalcTextSize(sub.c_str());
      dl->AddText(ImVec2(c.x - ss.x * 0.5f, c.y + ms.y + fs * 0.4f), ImGui::GetColorU32(kHudDim), sub.c_str());
    }
    const char* label = action == 1 ? ICON_FA_PLUG "  连接 CARLA" : action == 2 ? ICON_FA_PLUS "  生成主车"
                      : action == 3 ? ICON_FA_PLAY "  打开画面" : action == 4 ? ICON_FA_POWER_OFF "  启动后端" : nullptr;
    if (label) {
      const float bw = ImGui::CalcTextSize(label).x + fs * 2.0f;
      ImGui::SetCursorScreenPos(ImVec2(c.x - bw * 0.5f, c.y + ms.y + fs * 2.0f));
      ImGui::BeginDisabled(!busy_.empty() || (action == 1 && !be_.Connected()));
      const bool pressed = ui::Button("", label, ui::Kind::Primary, ImVec2(bw, 0));
      ui::RecordTarget("viewport:action");
      if (pressed) {
        if (action == 1) ConnectCarla();
        else if (action == 4) {
          if (backend_problem_.empty()) StartBackend();  // never started: nothing to recover
          else RestartBackend();
        }
        else if (action == 2) { panel_ = kPanelVehicle; monitor_open_ = true; SpawnEgo(); }
        else { view_auto_ = true; StartView(); }
      }
      ImGui::EndDisabled();
    }
  }

  // Camera bar (top-left): view selection.
  if (carla_connected_ && ego) {
    const float bar_h = fs * 1.9f;
    ImGui::SetCursorScreenPos(ImVec2(o.x + fs * 0.6f, o.y + fs * 0.6f));
    ImGui::PushStyleColor(ImGuiCol_Button, kHudBg);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.22f, 0.25f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.25f, 0.27f, 0.3f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, kHudBorder);
    ImGui::PushStyleColor(ImGuiCol_Text, kHudText);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
    static const char* kModes[] = {"chase", "hood", "wheel", "top"};
    static const char* kNames[] = {ICON_FA_CAR_REAR " 跟车", ICON_FA_EYE " 车头", ICON_FA_CIRCLE_DOT " 前轮", ICON_FA_ARROWS_TO_EYE " 俯视"};
    for (int i = 0; i < 4; ++i) {
      if (i) ImGui::SameLine();
      const bool on = view_on_ && view_rig_sensor_.empty() && view_mode_ == kModes[i];
      if (on) ImGui::PushStyleColor(ImGuiCol_Button, ui::WithAlpha(p.accent, 0.85f));
      ImGui::BeginDisabled(!busy_.empty());
      const bool pressed = ImGui::Button(kNames[i], ImVec2(0, bar_h));
      ui::RecordTarget(std::string("view:") + kModes[i]);
      if (pressed) {
        view_mode_ = kModes[i];
        view_auto_ = true;
        StartView();
      }
      ImGui::EndDisabled();
      if (on) ImGui::PopStyleColor();
    }
    if (view_on_) {
      ImGui::SameLine(0, fs * 0.5f);
      const bool pressed = ImGui::Button(ICON_FA_XMARK, ImVec2(bar_h, bar_h));
      ui::RecordTarget("view:close");
      if (pressed) {
        Call("view_stop", json::object(), nullptr);
        view_on_ = false;
        view_auto_ = false;
      }
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("关闭画面（节省带宽）");
    }
    // Layout: single view, one large + three small, 2 x 2.
    ImGui::SameLine(0, fs * 0.8f);
    static const char* kLayouts[] = {ICON_FA_SQUARE " 单画面", ICON_FA_TABLE_COLUMNS " 1+3", ICON_FA_TABLE_CELLS_LARGE " 2×2"};
    static const char* kLayoutTips[] = {"只显示主相机", "主相机 + 三个小视图（语义、点云、深度等，可点标签更换）",
                                        "四宫格：主相机 + 三个视图"};
    for (int i = 0; i < 3; ++i) {
      if (i) ImGui::SameLine();
      const bool on = view_layout_ == i;
      if (on) ImGui::PushStyleColor(ImGuiCol_Button, ui::WithAlpha(p.accent, 0.85f));
      ImGui::BeginDisabled(!busy_.empty());
      const bool pressed = ImGui::Button(kLayouts[i], ImVec2(0, bar_h));
      ui::RecordTarget(Fmt("layout:%d", i));
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", kLayoutTips[i]);
      if (pressed && view_layout_ != i) {
        view_layout_ = i;
        view_auto_ = true;
        SendViews();
      }
      ImGui::EndDisabled();
      if (on) ImGui::PopStyleColor();
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(5);
    // Camera caption (top-right)
    if (image) {
      const std::string cap = Fmt("%s  %d×%d  ·  %d 帧", view_rig_sensor_.empty() ? "主车相机" : view_rig_sensor_.c_str(),
                                  view_w_, view_h_, view_frames_);
      const ImVec2 cs = ImGui::CalcTextSize(cap.c_str());
      const float top = multi ? mo.y + fs * 3.0f : mo.y + fs * 0.6f;  // below the camera bar when panes are narrow
      const ImVec2 a(me.x - cs.x - fs * 1.6f, top);
      HudPanel(dl, a, ImVec2(me.x - fs * 0.6f, a.y + bar_h));
      dl->AddText(ImVec2(a.x + fs * 0.5f, a.y + (bar_h - fs) * 0.5f), ImGui::GetColorU32(kHudDim), cap.c_str());
    }
  }

  // Instrument cluster (bottom-left) and minimap (bottom-right).
  // Instruments only where the main pane is large (not in the 2x2 grid).
  if (carla_connected_ && ego && mh > fs * 16 && mw > fs * 20 && !(multi && view_layout_ == 2)) {
    DrawHud(ImVec2(mo.x + fs * 0.6f, me.y - fs * 0.6f));
    const float mm = std::min(fs * 13.0f, std::min(mh * 0.4f, mw * 0.3f));
    if (mw > fs * 40) DrawMinimap(ImVec2(me.x - fs * 0.6f - mm, me.y - fs * 0.6f - mm), mm);
  }
  float note_y = o.y + fs * 3.0f;
  if (!backend_problem_.empty()) {
    // The backend hung or died: the way out is a fresh one.
    const char* label = ICON_FA_POWER_OFF "  重启后端";
    const float lw = ImGui::CalcTextSize(label).x + fs * 1.6f;
    const float bw = std::min(w - fs * 2.0f, ImGui::CalcTextSize(backend_problem_.c_str()).x + lw + fs * 3.4f);
    const ImVec2 a(o.x + (w - bw) * 0.5f, note_y), b2(a.x + bw, a.y + fs * 2.2f);
    dl->AddRectFilled(a, b2, ImGui::GetColorU32(ImVec4(0.08f, 0.08f, 0.09f, 0.92f)), 4.0f);
    dl->AddRect(a, b2, ImGui::GetColorU32(p.danger), 4.0f, 0, 1.5f);
    dl->AddText(ImVec2(a.x + fs * 0.7f, a.y + fs * 0.6f), ImGui::GetColorU32(p.danger), ICON_FA_TRIANGLE_EXCLAMATION);
    dl->PushClipRect(a, ImVec2(b2.x - lw - fs * 0.8f, b2.y), true);
    dl->AddText(ImVec2(a.x + fs * 2.0f, a.y + fs * 0.6f), ImGui::GetColorU32(kHudText), backend_problem_.c_str());
    dl->PopClipRect();
    ImGui::SetCursorScreenPos(ImVec2(b2.x - lw - fs * 0.4f, a.y + fs * 0.3f));
    const bool pressed = ui::Button("", label, ui::Kind::Primary, ImVec2(lw, 0));
    ui::RecordTarget("viewport:restart_backend");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("结束当前后端（它的线程调用栈会记到 backend.prev.log），启动新的后端并重新连接 CARLA，\n"
                        "同时清理上一个后端留在 CARLA 里的主车、交通和传感器");
    if (pressed) RestartBackend();
    note_y += fs * 2.6f;
  }
  if (!run_note_.empty()) {
    // Banner under the camera bar: why the last run ended, dismissable.
    const ImVec4 col = run_note_level_ == "error" ? p.danger : p.warning;
    const float bw = std::min(w - fs * 2.0f, ImGui::CalcTextSize(run_note_.c_str()).x + fs * 4.0f);
    const ImVec2 a(o.x + (w - bw) * 0.5f, note_y), b2(a.x + bw, a.y + fs * 2.2f);
    dl->AddRectFilled(a, b2, ImGui::GetColorU32(ImVec4(0.08f, 0.08f, 0.09f, 0.9f)), 4.0f);
    dl->AddRect(a, b2, ImGui::GetColorU32(col), 4.0f, 0, 1.5f);
    dl->AddText(ImVec2(a.x + fs * 0.7f, a.y + fs * 0.6f), ImGui::GetColorU32(col),
                run_note_level_ == "error" ? ICON_FA_CIRCLE_XMARK : ICON_FA_CIRCLE_INFO);
    dl->PushClipRect(a, ImVec2(b2.x - fs * 2.0f, b2.y), true);
    dl->AddText(ImVec2(a.x + fs * 2.0f, a.y + fs * 0.6f), ImGui::GetColorU32(kHudText), run_note_.c_str());
    dl->PopClipRect();
    ImGui::SetCursorScreenPos(ImVec2(b2.x - fs * 1.8f, a.y + fs * 0.35f));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Text, kHudDim);
    if (ImGui::Button(ICON_FA_XMARK "##note")) run_note_.clear();
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("关闭提示");
  }
  if (Running() && last_tel_.value("at_red_light", false)) {
    const char* t = ICON_FA_TRAFFIC_LIGHT "  正在等红灯";
    const ImVec2 ts = ImGui::CalcTextSize(t);
    const ImVec2 a(o.x + (w - ts.x) * 0.5f - fs * 0.6f, o.y + fs * 3.0f);
    HudPanel(dl, a, ImVec2(a.x + ts.x + fs * 1.2f, a.y + fs * 1.8f));
    dl->AddText(ImVec2(a.x + fs * 0.6f, a.y + fs * 0.4f), ImGui::GetColorU32(p.danger), t);
  }
  ImGui::EndChild();
  ImGui::PopStyleColor();
}

// One extra viewport pane: image, source picker on its label, legend.
void App::DrawPane(int i, ImVec2 pos, ImVec2 size) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ViewPane& pane = panes_[i];
  const ImVec2 end(pos.x + size.x, pos.y + size.y);
  dl->AddRectFilled(pos, end, ImGui::GetColorU32(ImVec4(0.03f, 0.035f, 0.04f, 1)));
  if (pane.tex && pane.w > 0 && pane.h > 0 && pane.frames > 0) {
    const float s = std::min(size.x / pane.w, size.y / pane.h);
    const ImVec2 sz(pane.w * s, pane.h * s);
    const ImVec2 a(pos.x + (size.x - sz.x) * 0.5f, pos.y + (size.y - sz.y) * 0.5f);
    dl->AddImage(static_cast<ImTextureID>(static_cast<intptr_t>(pane.tex)), a, ImVec2(a.x + sz.x, a.y + sz.y));
  } else {
    const char* t = "正在等待画面 ...";
    const ImVec2 ts = ImGui::CalcTextSize(t);
    dl->AddText(ImVec2(pos.x + (size.x - ts.x) * 0.5f, pos.y + (size.y - ts.y) * 0.5f), ImGui::GetColorU32(kHudDim), t);
  }
  dl->AddRect(pos, end, ImGui::GetColorU32(ImVec4(1, 1, 1, 0.12f)));

  // Label = source picker.
  std::string label = pane.source;
  const auto sources = ViewSources();
  for (const auto& src : sources)
    if (src.first == pane.source) label = src.second;
  const std::string chip = label + "  " ICON_FA_CARET_DOWN;
  const ImVec2 cs = ImGui::CalcTextSize(chip.c_str());
  const ImVec2 ca(pos.x + fs * 0.4f, pos.y + fs * 0.4f), cb(ca.x + cs.x + fs * 1.0f, ca.y + fs * 1.6f);
  ImGui::SetCursorScreenPos(ca);
  ImGui::PushID(i);
  if (ImGui::InvisibleButton("##src", ImVec2(cb.x - ca.x, cb.y - ca.y))) ImGui::OpenPopup("srcpick");
  ui::RecordTarget(Fmt("pane:%d", i));
  const bool hov = ImGui::IsItemHovered();
  dl->AddRectFilled(ca, cb, ImGui::GetColorU32(hov ? ImVec4(0.2f, 0.22f, 0.25f, 0.9f) : kHudBg), 3.0f);
  dl->AddRect(ca, cb, ImGui::GetColorU32(kHudBorder), 3.0f);
  dl->AddText(ImVec2(ca.x + fs * 0.5f, ca.y + fs * 0.3f), ImGui::GetColorU32(kHudText), chip.c_str());
  if (hov) ImGui::SetTooltip("点击更换这个视图显示的内容");
  if (ImGui::BeginPopup("srcpick")) {
    for (const auto& src : sources) {
      if (ImGui::MenuItem(src.second.c_str(), nullptr, src.first == pane.source)) {
        pane.source = src.first;
        SendViews();
      }
      ui::RecordTarget("src:" + src.first);
    }
    ImGui::EndPopup();
  }
  ImGui::PopID();

  // Legends for the bird's-eye views.
  std::string kind = pane.source;
  if (kind.rfind("rig:", 0) == 0)
    for (const json& s : RigSensors())
      if ("rig:" + s.value("name", std::string()) == pane.source) kind = s.value("type", std::string());
  const char* legend = kind == "lidar" ? "俯视 · 车头朝上 · 圆环间隔 10 m · 颜色 = 高度（蓝低 红高）"
                     : kind == "radar" ? "俯视 · 车头朝上 · 圆环间隔 10 m · 相对速度：红 = 靠近  蓝 = 远离  白 = 无相对运动"
                     : kind == "semantic" ? "CityScapes 配色：路面紫 · 车辆蓝 · 行人红 · 植被绿"
                     : kind == "depth" ? "对数深度：越亮越远" : nullptr;
  if (legend && size.y > fs * 6) {
    const float ls = fs * 0.78f;
    const ImVec2 lsz = ImGui::GetFont()->CalcTextSizeA(ls, 1e9f, 0, legend);
    const ImVec2 la(pos.x + fs * 0.4f, end.y - fs * 0.4f - ls - fs * 0.5f);
    if (lsz.x + fs < size.x) {
      HudPanel(dl, la, ImVec2(la.x + lsz.x + fs * 0.8f, la.y + ls + fs * 0.5f));
      dl->AddText(ImGui::GetFont(), ls, ImVec2(la.x + fs * 0.4f, la.y + fs * 0.25f), ImGui::GetColorU32(kHudDim), legend);
    }
  }
  if (pane.frames > 0) {
    const std::string fr = Fmt("%d×%d", pane.w, pane.h);
    const float ls = fs * 0.75f;
    const float fw = ImGui::GetFont()->CalcTextSizeA(ls, 1e9f, 0, fr.c_str()).x;
    dl->AddText(ImGui::GetFont(), ls, ImVec2(end.x - fw - fs * 0.5f, pos.y + fs * 0.5f), ImGui::GetColorU32(kHudDim), fr.c_str());
  }
  (void)p;
}

// Speed readout, steering wheel, pedal bars. anchor = bottom-left corner.
void App::DrawHud(ImVec2 anchor) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const bool have = !last_tel_.empty();
  const float W = fs * 17.0f, H = fs * 6.4f;
  const ImVec2 a(anchor.x, anchor.y - H), b(anchor.x + W, anchor.y);
  HudPanel(dl, a, b);
  ImFont* big = ui::GetFonts().mono_big ? ui::GetFonts().mono_big : ImGui::GetFont();
  ImFont* mono = ui::GetFonts().mono ? ui::GetFonts().mono : ImGui::GetFont();

  // Speed
  const std::string sp = have ? Fmt("%.0f", last_tel_.value("speed_kmh", 0.0)) : std::string("-");
  const float big2 = big->FontSize * 1.45f;
  const ImVec2 ss = big->CalcTextSizeA(big2, 1e9f, 0, sp.c_str());
  dl->AddText(big, big2, ImVec2(a.x + fs * 5.2f - ss.x, a.y + fs * 0.9f), ImGui::GetColorU32(kHudText), sp.c_str());
  dl->AddText(ImGui::GetFont(), fs * 0.85f, ImVec2(a.x + fs * 5.4f, a.y + fs * 0.9f + big2 - fs * 1.1f),
              ImGui::GetColorU32(kHudDim), "km/h");
  const std::string tt = have ? Fmt("t %.2f s", last_tel_.value("t", 0.0)) : std::string("t - s");
  dl->AddText(mono, mono->FontSize, ImVec2(a.x + fs * 0.8f, b.y - fs * 1.5f), ImGui::GetColorU32(kHudDim), tt.c_str());

  // Steering wheel: angle in degrees, + = right.
  const json act = have ? last_tel_["action"] : json::array();
  const bool cosim = last_tel_.value("dynamics", std::string()) == "CarSim";
  const float sw_max = std::max(1.0f, cfg_.contains("sync") ? cfg_["sync"].value("steering_wheel_max_deg", 540.0f) : 540.0f);
  const float raw = act.size() > 2 ? static_cast<float>(appui::NumAt(act, 2)) : 0.0f;
  const float wheel_deg = cosim ? -raw : raw * sw_max;  // CarSim: deg, + left; CARLA: -1..1, + right
  const ImVec2 wc(a.x + fs * 10.2f, a.y + H * 0.45f);
  const float r = fs * 2.1f;
  dl->AddCircle(wc, r, ImGui::GetColorU32(kHudText), 48, 2.6f);
  dl->AddCircle(wc, r * 0.28f, ImGui::GetColorU32(kHudDim), 24, 1.5f);
  const float ang = wheel_deg * kPi / 180.0f;
  for (float base : {0.0f, kPi, kPi * 0.5f}) {  // right, left, bottom spokes (screen angles, y down)
    const float t = base + ang;
    dl->AddLine(ImVec2(wc.x + std::cos(t) * r * 0.28f, wc.y + std::sin(t) * r * 0.28f),
                ImVec2(wc.x + std::cos(t) * r, wc.y + std::sin(t) * r), ImGui::GetColorU32(kHudText), 2.2f);
  }
  const float top = -kPi * 0.5f + ang;
  dl->AddCircleFilled(ImVec2(wc.x + std::cos(top) * r, wc.y + std::sin(top) * r), fs * 0.22f, ImGui::GetColorU32(p.accent));
  const std::string wd = have ? Fmt("%+.0f°", wheel_deg) : std::string("-");
  const ImVec2 ws = ImGui::CalcTextSize(wd.c_str());
  dl->AddText(ImVec2(wc.x - ws.x * 0.5f, b.y - fs * 1.4f), ImGui::GetColorU32(kHudDim), wd.c_str());

  // Pedals
  const float thr = act.size() > 0 ? static_cast<float>(appui::NumAt(act, 0)) : 0.0f, brk = act.size() > 1 ? std::min(1.0f, static_cast<float>(appui::NumAt(act, 1))) : 0.0f;
  auto pedal = [&](float x, float v, const ImVec4& col, const char* label) {
    const float y0 = a.y + fs * 0.8f, y1 = b.y - fs * 1.6f, bw = fs * 0.75f;
    dl->AddRectFilled(ImVec2(x, y0), ImVec2(x + bw, y1), ImGui::GetColorU32(ImVec4(1, 1, 1, 0.08f)), 2.0f);
    const float fy = y1 - (y1 - y0) * std::max(0.0f, std::min(1.0f, v));
    dl->AddRectFilled(ImVec2(x, fy), ImVec2(x + bw, y1), ImGui::GetColorU32(col), 2.0f);
    const float ls = fs * 0.8f;
    const float lw = ImGui::GetFont()->CalcTextSizeA(ls, 1e9f, 0, label).x;
    dl->AddText(ImGui::GetFont(), ls, ImVec2(x + (bw - lw) * 0.5f, b.y - fs * 1.35f), ImGui::GetColorU32(kHudDim), label);
  };
  pedal(a.x + fs * 13.6f, thr, p.success, "油门");
  pedal(a.x + fs * 15.4f, brk, p.danger, "制动");
}

// Spawn points as the road skeleton, the ego trail and the ego heading.
// top-left corner + size; CARLA x to the right, y downwards.
void App::DrawMinimap(ImVec2 tl, float size) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 br(tl.x + size, tl.y + size);
  HudPanel(dl, tl, br);
  dl->AddText(ImGui::GetFont(), fs * 0.8f, ImVec2(tl.x + fs * 0.5f, tl.y + fs * 0.35f), ImGui::GetColorU32(kHudDim),
              ICON_FA_MAP "  地图");
  if (spawn_points_.empty()) return;
  float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f;
  for (const json& s : spawn_points_) {
    const float x = s.value("x", 0.0f), y = s.value("y", 0.0f);
    x0 = std::min(x0, x); x1 = std::max(x1, x); y0 = std::min(y0, y); y1 = std::max(y1, y);
  }
  const float pad = fs * 1.2f;
  const float span = std::max(1.0f, std::max(x1 - x0, y1 - y0));
  const float k = (size - 2 * pad) / span;
  const float cx = (x0 + x1) * 0.5f, cy = (y0 + y1) * 0.5f;
  auto map = [&](float x, float y) { return ImVec2(tl.x + size * 0.5f + (x - cx) * k, tl.y + size * 0.5f + fs * 0.3f + (y - cy) * k); };
  dl->PushClipRect(tl, br, true);
  for (const json& s : spawn_points_)
    dl->AddCircleFilled(map(s.value("x", 0.0f), s.value("y", 0.0f)), 1.6f, ImGui::GetColorU32(ImVec4(1, 1, 1, 0.28f)), 6);
  const int sp = cfg_.contains("carla") ? cfg_["carla"].value("spawn_index", 0) : 0;
  if (sp >= 0 && sp < static_cast<int>(spawn_points_.size())) {
    const json& s = spawn_points_[static_cast<size_t>(sp)];
    dl->AddCircle(map(s.value("x", 0.0f), s.value("y", 0.0f)), fs * 0.35f, ImGui::GetColorU32(p.warning), 16, 1.5f);
  }
  const size_t n = std::min(trail_x_.size(), trail_y_.size());
  if (n > 1) {
    std::vector<ImVec2> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; ++i) pts.push_back(map(trail_x_[i], trail_y_[i]));
    dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), ImGui::GetColorU32(p.accent), 0, 2.0f);
  }
  if (!last_tel_.empty()) {
    const json& loc = last_tel_["location"];
    const float yaw = static_cast<float>(appui::NumAt(last_tel_["rotation"], 1)) * kPi / 180.0f;
    const ImVec2 c = map(static_cast<float>(appui::NumAt(loc, 0)), static_cast<float>(appui::NumAt(loc, 1)));
    const float s = fs * 0.55f;
    const ImVec2 f(c.x + std::cos(yaw) * s, c.y + std::sin(yaw) * s);
    const ImVec2 l(c.x + std::cos(yaw + 2.5f) * s * 0.8f, c.y + std::sin(yaw + 2.5f) * s * 0.8f);
    const ImVec2 r(c.x + std::cos(yaw - 2.5f) * s * 0.8f, c.y + std::sin(yaw - 2.5f) * s * 0.8f);
    dl->AddTriangleFilled(f, l, r, ImGui::GetColorU32(ImVec4(1, 1, 1, 1)));
  }
  dl->PopClipRect();
}

// --------------------------------------------------------------------------
// Bottom dock: plots, vehicle state, output.
void App::DrawDock(float w, float h) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.panel);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.5f, fs * 0.3f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(7 * ui::Scale(), 6 * ui::Scale()));
  ImGui::BeginChild("dock", ImVec2(w, h), ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollbar);
  int errors = 0, warns = 0;
  for (const auto& l : log_) { errors += l.level == "error"; warns += l.level == "warn"; }
  const std::string out_label = errors ? Fmt(ICON_FA_TERMINAL "  输出  (%d 错误)###out", errors) : std::string(ICON_FA_TERMINAL "  输出###out");
  auto flags = [&](int i) { return dock_tab_select_ == i ? ImGuiTabItemFlags_SetSelected : 0; };
  if (ImGui::BeginTabBar("docktabs")) {
    if (ImGui::BeginTabItem(ICON_FA_CHART_LINE "  曲线", nullptr, flags(0))) {
      DrawPlots();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(ICON_FA_GAUGE_HIGH "  车辆状态", nullptr, flags(1))) {
      DrawVehicleState();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(out_label.c_str(), nullptr, flags(2))) {
      log_errors_ = 0;
      DrawLogList(warns, errors);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  dock_tab_select_ = -1;
  ImGui::EndChild();
  ImGui::PopStyleVar(2);
  ImGui::PopStyleColor();
}

void App::DrawPlots() {
  const ui::Palette& p = ui::Colors();
  const int n = static_cast<int>(h_t_.size());
  if (n < 2) {
    ImGui::TextColored(p.text_dim, "运行后显示车速、前轮转向角、悬架行程、油门 / 制动曲线");
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
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float gap = ImGui::GetStyle().ItemSpacing.x;
  const ImVec2 sz((avail.x - gap * 3) / 4.0f, avail.y);
  ImPlotFlags pf = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect;
  ImPlotSpec line;
  line.LineWeight = 1.5f;
  ImPlotAxisFlags ax = ImPlotAxisFlags_AutoFit;
  if (ImPlot::BeginPlot("车速 (km/h)", sz, pf | ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    fit(&h_speed_, 1, 10.0f, true);
    ImPlot::PlotLine("车速", h_t_.data(), h_speed_.data(), n, line);
    ImPlot::EndPlot();
  }
  ImGui::SameLine();
  if (ImPlot::BeginPlot("前轮转向角 (°)", sz, pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    const std::vector<float> steer[2] = {h_steer_fl_, h_steer_fr_};
    fit(steer, 2, 4.0f, false);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    ImPlot::PlotLine("左前", h_t_.data(), h_steer_fl_.data(), n, line);
    ImPlot::PlotLine("右前", h_t_.data(), h_steer_fr_.data(), n, line);
    ImPlot::EndPlot();
  }
  ImGui::SameLine();
  if (ImPlot::BeginPlot("悬架行程 (mm)", sz, pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    fit(h_susp_, 4, 10.0f, false);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    static const char* kW[] = {"左前", "右前", "左后", "右后"};
    for (int i = 0; i < 4; ++i) ImPlot::PlotLine(kW[i], h_t_.data(), h_susp_[i].data(), n, line);
    ImPlot::EndPlot();
  }
  ImGui::SameLine();
  if (ImPlot::BeginPlot("油门 / 制动", sz, pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -0.05, 1.05, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    ImPlot::PlotLine("油门", h_t_.data(), h_thr_.data(), n, line);
    ImPlot::PlotLine("制动", h_t_.data(), h_brk_.data(), n, line);
    ImPlot::EndPlot();
  }
}

void App::DrawVehicleState() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const bool have = !last_tel_.empty();
  ImGui::BeginChild("vstate", ImVec2(0, 0));
  if (ImGui::BeginTable("vs", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableNextRow();
    // Readouts
    ImGui::TableSetColumnIndex(0);
    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const float tw = (ImGui::GetContentRegionAvail().x - gap) / 2.0f;
    ui::KpiTile("车速", have ? Fmt("%.1f", last_tel_.value("speed_kmh", 0.0)).c_str() : "-", "km/h", tw);
    ImGui::SameLine();
    ui::KpiTile("仿真时间", have ? Fmt("%.2f", last_tel_.value("t", 0.0)).c_str() : "-", "s", tw);
    ui::KpiTile("实时倍率", have ? Fmt("%.2f", last_tel_.value("rt_factor", 0.0)).c_str() : "-", "x", tw);
    ImGui::SameLine();
    ui::KpiTile("帧", have ? Fmt("%d", last_tel_.value("frame", 0)).c_str() : "-", "", tw);
    // Controls
    ImGui::TableSetColumnIndex(1);
    ImGui::TextColored(p.text_dim, ICON_FA_SLIDERS "  控制输入");
    const json a = have ? last_tel_["action"] : json::array();
    const bool cosim = last_tel_.value("dynamics", std::string()) == "CarSim";
    const float thr = a.size() > 0 ? static_cast<float>(appui::NumAt(a, 0)) : 0.0f, brk = a.size() > 1 ? static_cast<float>(appui::NumAt(a, 1)) : 0.0f;
    const float sw = a.size() > 2 ? static_cast<float>(appui::NumAt(a, 2)) : 0.0f;
    const float steer = cosim ? -sw / std::max(1.0f, cfg_["sync"].value("steering_wheel_max_deg", 540.0f)) : sw;
    ControlBar("油门", thr, 0, 1, p.success, false);
    ControlBar("制动", std::min(1.0f, brk), 0, 1, p.danger, false);
    ControlBar("转向", steer, -1, 1, p.accent, true);
    if (collect_stats_.contains("frames")) {
      ImGui::TextColored(p.text_dim, ICON_FA_DATABASE "  采集 %d 帧  %.1f MB", collect_stats_.value("frames", 0),
                         collect_stats_.value("bytes", 0.0) / 1e6);
      const int maxf = cfg_["collect"].value("max_frames", 0);
      if (maxf > 0) ImGui::ProgressBar(std::min(1.0f, collect_stats_.value("frames", 0) / static_cast<float>(maxf)), ImVec2(-1, fs * 0.5f), "");
    }
    // Pose
    ImGui::TableSetColumnIndex(2);
    ImGui::TextColored(p.text_dim, ICON_FA_LOCATION_CROSSHAIRS "  位姿");
    if (ImGui::BeginTable("pose", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_RowBg)) {
      const char* names[] = {"X m", "Y m", "Z m", "航向 °", "俯仰 °", "侧倾 °"};
      double vals[6] = {0, 0, 0, 0, 0, 0};
      if (have) {
        const json& loc = last_tel_["location"];
        const json& rot = last_tel_["rotation"];
        const double v[] = {appui::NumAt(loc, 0), appui::NumAt(loc, 1), appui::NumAt(loc, 2),
                            appui::NumAt(rot, 1), appui::NumAt(rot, 0), appui::NumAt(rot, 2)};
        std::copy(v, v + 6, vals);
      }
      for (int i = 0; i < 6; ++i) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextColored(p.text_dim, "%s", names[i]);
        ImGui::TableSetColumnIndex(1);
        if (ui::GetFonts().mono) ImGui::PushFont(ui::GetFonts().mono);
        if (have) ImGui::Text("%10.2f", vals[i]); else ImGui::TextDisabled("%10s", "-");
        if (ui::GetFonts().mono) ImGui::PopFont();
      }
      ImGui::EndTable();
    }
    // Wheels
    ImGui::TableSetColumnIndex(3);
    ImGui::TextColored(p.text_dim, ICON_FA_CIRCLE_DOT "  车轮");
    if (ImGui::BeginTable("wheels", 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame)) {
      ImGui::TableSetupColumn("");
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
        ImGui::TableSetColumnIndex(1); if (i < st.size()) ImGui::Text("%+7.2f", appui::NumAt(st, i)); else ImGui::TextDisabled("   -");
        ImGui::TableSetColumnIndex(2); if (i < ro.size()) ImGui::Text("%6.0f", appui::NumAt(ro, i)); else ImGui::TextDisabled("   -");
        ImGui::TableSetColumnIndex(3); if (i < su.size()) ImGui::Text("%+6.1f", appui::NumAt(su, i)); else ImGui::TextDisabled("   -");
        if (ui::GetFonts().mono) ImGui::PopFont();
      }
      ImGui::EndTable();
    }
    ImGui::EndTable();
  }
  ImGui::EndChild();
}

void App::DrawLogList(int warns, int errors) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 1));
  const char* names[] = {"全部", "警告", "错误"};
  const int counts[] = {static_cast<int>(log_.size()), warns, errors};
  for (int i = 0; i < 3; ++i) {
    if (i) ImGui::SameLine(0, 2);
    ImGui::PushStyleColor(ImGuiCol_Button, log_filter_ == i ? ui::WithAlpha(p.accent, 0.3f) : ImVec4(0, 0, 0, 0));
    if (ImGui::SmallButton(Fmt("%s %d##lf%d", names[i], counts[i], i).c_str())) log_filter_ = i;
    ImGui::PopStyleColor();
  }
  ImGui::SameLine(0, fs);
  if (ImGui::SmallButton(ICON_FA_TRASH " 清空")) { log_.clear(); log_errors_ = 0; }
  ImGui::PopStyleVar();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.field);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.5f, fs * 0.25f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 2));
  ImGui::BeginChild("logscroll", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding);
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
  ImGui::PopStyleColor();
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
    log_open_ = true;
    dock_tab_select_ = 2;
    log_errors_ = 0;
  }
  ImGui::PopStyleColor(3);
  ImGui::SameLine();
  ImGui::TextColored(dim, "%.0f fps", ImGui::GetIO().Framerate);
  ImGui::EndChild();
  ImGui::PopStyleVar(2);
  ImGui::PopStyleColor();
}
