// Window shell: toolbar, navigation, monitor column, status bar, log drawer.
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
};
struct NavGroup {
  const char* caption;
  std::vector<NavItem> items;
};

const std::vector<NavGroup>& Nav() {
  static const std::vector<NavGroup> g = {
      {"开始", {{kPanelConnect, ICON_FA_PLUG, "连接"}}},
      {"场景", {{kPanelWorld, ICON_FA_EARTH_ASIA, "地图与天气"}, {kPanelTraffic, ICON_FA_TRAFFIC_LIGHT, "交通流"},
                {kPanelActors, ICON_FA_LAYER_GROUP, "场景对象"}}},
      {"车辆", {{kPanelVehicle, ICON_FA_CAR_SIDE, "车辆与视角"}, {kPanelRig, ICON_FA_SATELLITE_DISH, "传感器套件"}}},
      {"仿真", {{kPanelDrive, ICON_FA_ROBOT, "驾驶模式"}, {kPanelCoSim, ICON_FA_GEARS, "CarSim 动力学"}}},
      {"数据", {{kPanelCollect, ICON_FA_DATABASE, "数据采集"}, {kPanelRecorder, ICON_FA_FILM, "录制与回放"},
                {kPanelView, ICON_FA_VIDEO, "实时画面"}}},
  };
  return g;
}

const char* DriverName(const json& cfg) {
  if (!cfg.contains("drive")) return "-";
  const json& d = cfg["drive"];
  const bool cosim = d.value("dynamics", std::string("cosim")) == "cosim";
  const std::string v = cosim ? d.value("cosim_driver", std::string("demo")) : d.value("carla_driver", std::string("route"));
  if (v == "demo") return "演示";
  if (v == "pid") return "PID 跟踪";
  if (v == "route") return "路线跟随";
  if (v == "manual") return "键盘驾驶";
  if (v == "autopilot") return "CARLA 自动驾驶";
  return "-";
}

void ControlBar(const char* label, float v, float lo, float hi, const ImVec4& col, bool centered) {
  ImGui::PushStyleColor(ImGuiCol_Text, ui::Colors().text_dim);
  ImGui::TextUnformatted(label);
  ImGui::PopStyleColor();
  ImGui::SameLine(ImGui::GetFontSize() * 3.2f);
  ImVec2 p = ImGui::GetCursorScreenPos();
  const float w = ImGui::GetContentRegionAvail().x - ImGui::GetFontSize() * 3.0f;
  const float h = ImGui::GetTextLineHeight() * 0.6f;
  const float y = p.y + (ImGui::GetTextLineHeight() - h) * 0.5f;
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + w, y + h), ImGui::GetColorU32(ImGuiCol_FrameBg), h * 0.5f);
  const float t = std::max(0.0f, std::min(1.0f, (v - lo) / (hi - lo)));
  if (centered) {
    const float mid = p.x + w * 0.5f, x = p.x + w * t;
    dl->AddRectFilled(ImVec2(std::min(mid, x), y), ImVec2(std::max(mid, x), y + h), ImGui::GetColorU32(col), h * 0.5f);
    dl->AddLine(ImVec2(mid, y - 2), ImVec2(mid, y + h + 2), ImGui::GetColorU32(ui::Colors().text_dim));
  } else {
    dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + w * t, y + h), ImGui::GetColorU32(col), h * 0.5f);
  }
  ImGui::Dummy(ImVec2(w, ImGui::GetTextLineHeight()));
  ImGui::SameLine();
  ImGui::Text("%+.2f", v);
}

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
  if (carla_connected_ && frame_ % 600 == 0 && !Running()) RefreshDisk();

  const ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(vp->WorkSize);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin("##root", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
  ImGui::PopStyleVar();

  DrawToolbar();
  const float fs = ImGui::GetFontSize();
  const float status_h = ImGui::GetFrameHeight() + 6.0f;
  const float drawer_h = log_open_ ? fs * 11.0f : 0.0f;
  const float body_h = ImGui::GetContentRegionAvail().y - status_h - drawer_h;
  const float nav_w = fs * 11.5f, mon_w = fs * 27.0f;

  ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::Colors().panel);
  ImGui::BeginChild("nav", ImVec2(nav_w, body_h), ImGuiChildFlags_None);
  DrawNav();
  ImGui::EndChild();
  ImGui::PopStyleColor();
  ImGui::SameLine(0, 0);

  ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::Colors().bg);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 1.0f, fs * 0.8f));
  ImGui::BeginChild("panel", ImVec2(ImGui::GetContentRegionAvail().x - mon_w, body_h),
                    ImGuiChildFlags_AlwaysUseWindowPadding);
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
  ImGui::SameLine(0, 0);

  ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::Colors().panel);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.8f, fs * 0.7f));
  ImGui::BeginChild("monitor", ImVec2(0, body_h), ImGuiChildFlags_AlwaysUseWindowPadding);
  DrawMonitor();
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();

  if (log_open_) DrawLogDrawer();
  DrawStatusBar();
  ImGui::End();
}

// --------------------------------------------------------------------------
void App::DrawToolbar() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const float h = fs * 3.1f;
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::IsDark() ? ImVec4(0.07f, 0.08f, 0.10f, 1) : ImVec4(0.86f, 0.89f, 0.93f, 1));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.9f, (h - ImGui::GetFrameHeight() * 1.25f) * 0.5f));
  ImGui::BeginChild("toolbar", ImVec2(0, h), ImGuiChildFlags_AlwaysUseWindowPadding,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

  // Brand
  if (ui::GetFonts().bold) ImGui::PushFont(ui::GetFonts().bold);
  ImGui::AlignTextToFramePadding();
  ImGui::TextColored(p.accent, ICON_FA_CAR);
  ImGui::SameLine();
  ImGui::TextUnformatted("CARLA CoSim Studio");
  if (ui::GetFonts().bold) ImGui::PopFont();
  ImGui::SameLine(0, fs * 2.0f);

  // Run controls
  const bool active = Running();
  const ImVec2 big(fs * 5.2f, ImGui::GetFrameHeight() * 1.25f);
  ImGui::BeginDisabled(active || !carla_connected_ || !busy_.empty());
  if (ui::Button(ICON_FA_PLAY, "运行", ui::Kind::Success, big)) StartRun();
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (run_state_ == "paused") {
    if (ui::Button(ICON_FA_PLAY, "继续", ui::Kind::Primary, big)) RunCommand("cosim_resume");
  } else {
    ImGui::BeginDisabled(run_state_ != "running");
    if (ui::Button(ICON_FA_PAUSE, "暂停", ui::Kind::Secondary, big)) RunCommand("cosim_pause");
    ImGui::EndDisabled();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(run_state_ != "paused");
  if (ui::Button(ICON_FA_FORWARD_STEP, "单步", ui::Kind::Secondary, big)) RunCommand("cosim_step");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!active);
  if (ui::Button(ICON_FA_STOP, "停止", ui::Kind::Danger, big)) RunCommand("cosim_stop");
  ImGui::EndDisabled();
  ImGui::SameLine(0, fs * 1.5f);

  // State + run summary
  ImVec4 col = run_state_ == "running" ? p.success : run_state_ == "paused" ? p.warning
             : run_state_ == "error" ? p.danger : p.text_dim;
  const char* st = run_state_ == "running" ? "运行中" : run_state_ == "paused" ? "已暂停"
                 : run_state_ == "finished" ? "已完成" : run_state_ == "error" ? "出错" : "空闲";
  ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (big.y - ImGui::GetFrameHeight()) * 0.5f);
  ui::Pill(st, col);
  ImGui::SameLine();
  ImGui::BeginGroup();
  const bool cosim = cfg_.contains("drive") && cfg_["drive"].value("dynamics", std::string("cosim")) == "cosim";
  const bool collect = cfg_.contains("collect") && cfg_["collect"].value("enabled", false);
  ImGui::TextColored(p.text_dim, "%s %s   %s %s   %s %s", ICON_FA_GEARS, cosim ? "CarSim 动力学" : "CARLA 物理",
                     ICON_FA_ROBOT, DriverName(cfg_), ICON_FA_DATABASE, collect ? "采集开启" : "不采集");
  if (!last_tel_.empty()) {
    const int nf = last_tel_.value("n_frames", 0);
    ImGui::Text("t = %.2f s%s   %.1f km/h   %.2fx 实时", last_tel_.value("t", 0.0),
                nf > 0 ? Fmt(" / %.1f s", nf * cfg_["sync"].value("frame_dt", 0.02)).c_str() : "",
                last_tel_.value("speed_kmh", 0.0), last_tel_.value("rt_factor", 0.0));
  } else {
    ImGui::TextColored(p.text_dim, "%s", carla_connected_ ? "就绪：点“运行”开始仿真" : "先在“连接”页连接 CARLA");
  }
  ImGui::EndGroup();

  // Right side: theme + save
  const float right = ImGui::GetWindowContentRegionMax().x;
  ImGui::SameLine(right - fs * 5.6f);
  ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (big.y - ImGui::GetFrameHeight()) * 0.5f);
  if (ui::IconButton(dark_ ? ICON_FA_SUN : ICON_FA_MOON, dark_ ? "切换浅色主题" : "切换深色主题", "theme")) {
    dark_ = !dark_;
    theme_changed_ = true;
    SavePrefs();
  }
  ImGui::SameLine();
  if (ui::IconButton(ICON_FA_FLOPPY_DISK, "保存当前配置", "save"))
    SaveConfig(cfg_path_.empty() ? std::string("cosim_config.json") : cfg_path_);
  ImGui::SameLine();
  if (ui::IconButton(ICON_FA_TERMINAL, "日志", "log")) log_open_ = !log_open_;

  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}

// --------------------------------------------------------------------------
void App::DrawNav() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::Dummy(ImVec2(0, fs * 0.3f));
  for (const auto& g : Nav()) {
    ImGui::Indent(fs * 1.1f);
    ui::SectionCaption(g.caption);
    ImGui::Unindent(fs * 1.1f);
    for (const auto& it : g.items) {
      const bool needs_carla = it.panel != kPanelConnect;
      const bool disabled = needs_carla && !carla_connected_;
      const bool sel = panel_ == it.panel;
      ImGui::PushID(it.panel);
      ImVec2 pos = ImGui::GetCursorScreenPos();
      const float w = ImGui::GetContentRegionAvail().x, h = fs * 2.2f;
      ImGui::BeginDisabled(disabled);
      if (ImGui::InvisibleButton("##nav", ImVec2(w, h))) panel_ = it.panel;
      const bool hov = ImGui::IsItemHovered();
      ImGui::EndDisabled();
      ImDrawList* dl = ImGui::GetWindowDrawList();
      if (sel || hov) {
        ImVec4 bg = p.accent;
        bg.w = sel ? 0.16f : 0.07f;
        dl->AddRectFilled(ImVec2(pos.x + fs * 0.4f, pos.y), ImVec2(pos.x + w - fs * 0.4f, pos.y + h),
                          ImGui::GetColorU32(bg), fs * 0.35f);
      }
      if (sel) dl->AddRectFilled(ImVec2(pos.x + fs * 0.4f, pos.y + h * 0.22f), ImVec2(pos.x + fs * 0.55f, pos.y + h * 0.78f),
                                 ImGui::GetColorU32(p.accent), 2.0f);
      ImVec4 tc = disabled ? p.text_dim : (sel ? p.text : p.text);
      ImVec4 ic = disabled ? p.text_dim : (sel ? p.accent : p.text_dim);
      const float ty = pos.y + (h - ImGui::GetTextLineHeight()) * 0.5f;
      dl->AddText(ImVec2(pos.x + fs * 1.1f, ty), ImGui::GetColorU32(ic), it.icon);
      ImFont* f = sel && ui::GetFonts().bold ? ui::GetFonts().bold : ImGui::GetFont();
      dl->AddText(f, ImGui::GetFontSize(), ImVec2(pos.x + fs * 2.7f, ty), ImGui::GetColorU32(tc), it.name);
      ImGui::PopID();
    }
  }
}

// --------------------------------------------------------------------------
void App::DrawMonitor() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();

  if (view_on_ && view_tex_) {
    ui::BeginCard(ICON_FA_VIDEO, "实时画面", "mon_view");
    DrawViewImage(ImGui::GetContentRegionAvail().x, fs * 12.0f);
    ui::EndCard();
  }

  ui::BeginCard(ICON_FA_GAUGE_HIGH, "车辆状态", "mon_state");
  const float tile_w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2) / 3.0f;
  const bool have = !last_tel_.empty();
  ui::KpiTile("车速", have ? Fmt("%.1f", last_tel_.value("speed_kmh", 0.0)).c_str() : "--", "km/h", tile_w);
  ImGui::SameLine();
  ui::KpiTile("仿真时间", have ? Fmt("%.1f", last_tel_.value("t", 0.0)).c_str() : "--", "s", tile_w);
  ImGui::SameLine();
  ui::KpiTile("实时倍率", have ? Fmt("%.2f", last_tel_.value("rt_factor", 0.0)).c_str() : "--", "x", tile_w);
  if (have) {
    const json& a = last_tel_["action"];
    const bool cosim = last_tel_.value("dynamics", std::string()) == "CarSim";
    const float thr = a.size() > 0 ? a[0].get<float>() : 0.0f, brk = a.size() > 1 ? a[1].get<float>() : 0.0f;
    // CarSim action[2] is the steering wheel angle (deg, + left); CARLA's is steer (-1..1, + right).
    const float sw = a.size() > 2 ? a[2].get<float>() : 0.0f;
    const float steer = cosim ? -sw / std::max(1.0f, cfg_["sync"].value("steering_wheel_max_deg", 540.0f)) : sw;
    ControlBar("油门", thr, 0, 1, p.success, false);
    ControlBar("制动", std::min(1.0f, brk), 0, 1, p.danger, false);
    ControlBar("转向", steer, -1, 1, p.accent, true);
    if (last_tel_.value("at_red_light", false)) {
      ImGui::TextColored(p.danger, ICON_FA_TRAFFIC_LIGHT "  正在等红灯");
    }
    const json& loc = last_tel_["location"];
    const json& rot = last_tel_["rotation"];
    ImGui::TextColored(p.text_dim, "位置 %.1f, %.1f, %.2f   航向 %.1f°   俯仰 %.2f°   侧倾 %.2f°",
                       loc[0].get<double>(), loc[1].get<double>(), loc[2].get<double>(), rot[1].get<double>(),
                       rot[0].get<double>(), rot[2].get<double>());
    if (ImGui::BeginTable("wheels", 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("车轮");
      ImGui::TableSetupColumn("转向 °");
      ImGui::TableSetupColumn("转角 °");
      ImGui::TableSetupColumn("悬架 mm");
      ImGui::TableHeadersRow();
      static const char* kW[] = {"左前", "右前", "左后", "右后"};
      const json& st = last_tel_["wheel_steer"];
      const json& ro = last_tel_["wheel_rotation"];
      const json& su = last_tel_["wheel_suspension_mm"];
      for (size_t i = 0; i < 4; ++i) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(kW[i]);
        ImGui::TableSetColumnIndex(1); if (i < st.size()) ImGui::Text("%+.2f", st[i].get<double>()); else ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(2); if (i < ro.size()) ImGui::Text("%.0f", ro[i].get<double>()); else ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(3); if (i < su.size()) ImGui::Text("%+.1f", su[i].get<double>()); else ImGui::TextDisabled("-");
      }
      ImGui::EndTable();
    }
  } else {
    ImGui::TextColored(p.text_dim, "运行后显示实时数据");
  }
  ui::EndCard();

  if (collect_stats_.contains("frames")) {
    ui::BeginCard(ICON_FA_DATABASE, "采集进度", "mon_collect");
    ImGui::Text("%d 帧   %.1f MB   %.1f MB/s", collect_stats_.value("frames", 0),
                collect_stats_.value("bytes", 0.0) / 1e6, collect_stats_.value("mb_per_s", 0.0));
    const int maxf = cfg_["collect"].value("max_frames", 0);
    if (maxf > 0)
      ImGui::ProgressBar(std::min(1.0f, collect_stats_.value("frames", 0) / static_cast<float>(maxf)), ImVec2(-1, 0));
    if (collect_stats_.value("done", false))
      ImGui::TextColored(p.success, ICON_FA_CIRCLE_CHECK "  %s", collect_stats_.value("reason", std::string()).c_str());
    ui::EndCard();
  }

  ui::BeginCard(ICON_FA_CHART_LINE, "曲线", "mon_plots");
  const int n = static_cast<int>(h_t_.size());
  if (n < 2) {
    ImGui::TextColored(p.text_dim, "运行后显示车速、前轮转角、悬架行程、油门 / 制动曲线");
    ui::EndCard();
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
  const float ph = fs * 7.2f;
  ImPlotFlags pf = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect;
  ImPlotSpec line;
  line.LineWeight = 1.6f;
  ImPlotAxisFlags ax = ImPlotAxisFlags_AutoFit;
  if (ImPlot::BeginPlot("车速 (km/h)", ImVec2(-1, ph), pf | ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    fit(&h_speed_, 1, 10.0f, true);
    if (n) ImPlot::PlotLine("车速", h_t_.data(), h_speed_.data(), n, line);
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("前轮转向角 (°)", ImVec2(-1, ph), pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    const std::vector<float> steer[2] = {h_steer_fl_, h_steer_fr_};
    fit(steer, 2, 4.0f, false);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    if (n) {
      ImPlot::PlotLine("左前", h_t_.data(), h_steer_fl_.data(), n, line);
      ImPlot::PlotLine("右前", h_t_.data(), h_steer_fr_.data(), n, line);
    }
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("悬架行程 (mm)", ImVec2(-1, ph), pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    fit(h_susp_, 4, 10.0f, false);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    static const char* kW[] = {"左前", "右前", "左后", "右后"};
    if (n)
      for (int i = 0; i < 4; ++i) ImPlot::PlotLine(kW[i], h_t_.data(), h_susp_[i].data(), n, line);
    ImPlot::EndPlot();
  }
  if (ImPlot::BeginPlot("油门 / 制动", ImVec2(-1, ph), pf)) {
    ImPlot::SetupAxes(nullptr, nullptr, ax, ImPlotAxisFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -0.05, 1.05, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    if (n) {
      ImPlot::PlotLine("油门", h_t_.data(), h_thr_.data(), n, line);
      ImPlot::PlotLine("制动", h_t_.data(), h_brk_.data(), n, line);
    }
    ImPlot::EndPlot();
  }
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawStatusBar() {
  const ui::Palette& p = ui::Colors();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::IsDark() ? ImVec4(0.07f, 0.08f, 0.10f, 1) : ImVec4(0.86f, 0.89f, 0.93f, 1));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ImGui::GetFontSize() * 0.8f, 3));
  ImGui::BeginChild("status", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollbar);
  ImGui::AlignTextToFramePadding();
  const bool be = be_.Connected();
  ui::StatusDot(be ? p.success : (plat::IsAlive(backend_proc_) ? p.warning : p.danger));
  ImGui::SameLine(0, 2);
  ImGui::TextColored(p.text_dim, "后端 %s", be ? "已连接" : (plat::IsAlive(backend_proc_) ? "启动中" : "未运行"));
  ImGui::SameLine(0, 18);
  ui::StatusDot(carla_connected_ ? p.success : p.danger);
  ImGui::SameLine(0, 2);
  if (carla_connected_)
    ImGui::TextColored(p.text_dim, "CARLA %s  %s  %s", server_info_.value("server_version", std::string()).c_str(),
                       ICON_FA_MAP, world_.value("map", std::string()).c_str());
  else
    ImGui::TextColored(p.text_dim, "CARLA 未连接");
  if (world_.value("ego_id", 0)) {
    ImGui::SameLine(0, 18);
    ImGui::TextColored(p.text_dim, "%s 主车 #%d", ICON_FA_CAR, world_.value("ego_id", 0));
  }
  if (disk_.contains("free_gb")) {
    ImGui::SameLine(0, 18);
    const double free_gb = disk_.value("free_gb", 0.0);
    ImGui::TextColored(free_gb < 20 ? p.warning : p.text_dim, "%s 剩余 %.0f GB", ICON_FA_DATABASE, free_gb);
  }
  if (!busy_.empty()) {
    ImGui::SameLine(0, 18);
    const char* spin = "|/-\\";
    ImGui::TextColored(p.warning, "%c %s", spin[(frame_ / 8) % 4], busy_.c_str());
  }
  const float right = ImGui::GetWindowContentRegionMax().x;
  ImGui::SameLine(right - ImGui::GetFontSize() * 8.5f);
  std::string label = log_errors_ ? Fmt(ICON_FA_TRIANGLE_EXCLAMATION " 日志 (%d)", log_errors_) : std::string(ICON_FA_TERMINAL " 日志");
  ImGui::PushStyleColor(ImGuiCol_Text, log_errors_ ? p.warning : p.text_dim);
  if (ImGui::SmallButton(label.c_str())) {
    log_open_ = !log_open_;
    log_errors_ = 0;
  }
  ImGui::PopStyleColor();
  ImGui::SameLine();
  ImGui::TextColored(p.text_dim, "%.0f fps", ImGui::GetIO().Framerate);
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}

void App::DrawLogDrawer() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, p.panel);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(fs * 0.8f, fs * 0.4f));
  ImGui::BeginChild("logdrawer", ImVec2(0, fs * 11.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding);
  if (ImGui::SmallButton(ICON_FA_TRASH " 清空")) log_.clear();
  ImGui::SameLine();
  if (ImGui::SmallButton(ICON_FA_XMARK " 收起")) log_open_ = false;
  ImGui::BeginChild("logscroll");
  for (const auto& l : log_) {
    const ImVec4 c = l.level == "error" ? p.danger : l.level == "warn" ? p.warning : p.text;
    const char* icon = l.level == "error" ? ICON_FA_TRIANGLE_EXCLAMATION : l.level == "warn" ? ICON_FA_CIRCLE_INFO : " ";
    ImGui::TextColored(c, "%s %s", icon, l.text.c_str());
  }
  if (log_scroll_) {
    ImGui::SetScrollHereY(1.0f);
    log_scroll_ = false;
  }
  ImGui::EndChild();
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}
