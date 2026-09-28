// 运行对比: the run records of a record dir (the backend's runs_list), up to
// four picked side by side: key figures on the page, their trajectories and
// time series overlaid in the dockable 对比 panel (run_series).
#include <algorithm>
#include <cmath>
#include <filesystem>

#include "app.h"
#include "implot.h"
#include "ui_kit.h"

using appui::Fmt;
namespace fs = std::filesystem;

namespace {
const ImVec4 kRunColors[] = {ImVec4(0.30f, 0.62f, 1.00f, 1), ImVec4(1.00f, 0.55f, 0.20f, 1), ImVec4(0.35f, 0.85f, 0.45f, 1),
                             ImVec4(0.85f, 0.40f, 0.95f, 1)};
const char* kLetters[] = {"A", "B", "C", "D"};

double Num(const json& j, const char* k, double def = NAN) {
  if (!j.is_object() || !j.contains(k) || !j[k].is_number()) return def;
  return j[k].get<double>();
}

// The value of series y (over times t) at time x: interpolated, NaN outside or between missing values.
double ValueAt(const std::vector<float>& t, const std::vector<float>& y, double x) {
  const size_t n = std::min(t.size(), y.size());
  if (n == 0 || x < t[0] || x > t[n - 1]) return NAN;
  size_t i = static_cast<size_t>(std::upper_bound(t.begin(), t.begin() + static_cast<long>(n), static_cast<float>(x)) - t.begin());
  if (i == 0) return y[0];
  if (i >= n) return y[n - 1];
  const double t0 = t[i - 1], t1 = t[i], k = t1 > t0 ? (x - t0) / (t1 - t0) : 0.0;
  return y[i - 1] + k * (y[i] - y[i - 1]);
}

// A json array of numbers / nulls -> floats (NaN for null).
std::vector<float> Floats(const json& a) {
  std::vector<float> v;
  if (!a.is_array()) return v;
  v.reserve(a.size());
  for (const json& x : a) v.push_back(x.is_number() ? x.get<float>() : NAN);
  return v;
}
}  // namespace

void App::RunsRefresh() {
  if (runs_path_.empty()) runs_path_ = cfg_.contains("run") ? cfg_["run"].value("log_path", std::string("runs")) : std::string("runs");
  if (runs_path_.empty()) runs_path_ = "runs";
  Call("runs_list", {{"path", runs_path_}}, [this](const json& r) {
    runs_list_ = r;
    // Picked runs that are gone (deleted, another folder): dropped.
    std::vector<std::string> keep;
    for (const auto& f : runs_sel_)
      for (const json& x : r.value("runs", json::array()))
        if (x.value("folder", std::string()) == f) keep.push_back(f);
    runs_sel_ = keep;
  });
}

void App::RunsToggle(const std::string& folder) {
  auto it = std::find(runs_sel_.begin(), runs_sel_.end(), folder);
  if (it != runs_sel_.end()) {
    runs_sel_.erase(it);
    return;
  }
  if (runs_sel_.size() >= 4) runs_sel_.erase(runs_sel_.begin());  // the first picked goes
  runs_sel_.push_back(folder);
  if (!runs_series_.count(folder)) {
    ++runs_pending_;
    be_.Request("run_series", {{"folder", folder}}, [this, folder](bool ok, const json& r, const std::string& err) {
      --runs_pending_;
      if (!ok) {
        Log("读不了这次运行的记录：" + err, "error");
        auto k = std::find(runs_sel_.begin(), runs_sel_.end(), folder);
        if (k != runs_sel_.end()) runs_sel_.erase(k);
        return;
      }
      runs_series_[folder] = r;
    });
  }
  compare_open_ = true;
}

void App::DrawPanelRuns() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  if (runs_list_.empty() && be_.Connected() && busy_.empty()) RunsRefresh();

  ui::BeginCard(ICON_FA_LIST, "运行记录");
  ui::Row("目录", "“驾驶模式”页“运行记录目录”（相对桥接目录）；每次运行一个文件夹");
  char buf[512];
  std::snprintf(buf, sizeof(buf), "%s", runs_path_.c_str());
  ImGui::SetNextItemWidth(-fs * 5.5f);
  if (ImGui::InputText("##runspath", buf, sizeof(buf), ImGuiInputTextFlags_EnterReturnsTrue)) {
    runs_path_ = buf;
    RunsRefresh();
  }
  ImGui::SameLine();
  if (ui::Button(ICON_FA_ROTATE_RIGHT, "刷新")) RunsRefresh();
  ui::RecordTarget("runs:refresh");
  const std::string err = runs_list_.value("error", std::string());
  if (!err.empty()) ImGui::TextColored(p.text_dim, "%s", err.c_str());
  const json runs = runs_list_.value("runs", json::array());
  ui::DimWrapped("勾选最多 4 次运行（再勾第 5 次时，最早勾的那次取消）：下面并排比较指标，“对比”面板叠画轨迹和曲线。");
  // Fixed column widths, scrolled sideways when the panel is narrow (not squeezed to a letter).
  if (ImGui::BeginTable("##runs", 8, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                         ImGuiTableFlags_ScrollX | ImGuiTableFlags_SizingFixedFit, ImVec2(0, fs * 16))) {
    ImGui::TableSetupScrollFreeze(1, 1);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, fs * 2.2f);
    ImGui::TableSetupColumn("时间", ImGuiTableColumnFlags_WidthFixed, fs * 7.5f);
    ImGui::TableSetupColumn("算法", ImGuiTableColumnFlags_WidthFixed, fs * 9.0f);
    ImGui::TableSetupColumn("车辆", ImGuiTableColumnFlags_WidthFixed, fs * 5.5f);
    ImGui::TableSetupColumn("地图", ImGuiTableColumnFlags_WidthFixed, fs * 5.0f);
    ImGui::TableSetupColumn("时长 s", ImGuiTableColumnFlags_WidthFixed, fs * 3.5f);
    ImGui::TableSetupColumn("偏差 m", ImGuiTableColumnFlags_WidthFixed, fs * 3.5f);
    ImGui::TableSetupColumn("碰撞", ImGuiTableColumnFlags_WidthFixed, fs * 2.5f);
    ImGui::TableHeadersRow();
    int i = 0;
    for (const json& r : runs) {
      const std::string folder = r.value("folder", std::string());
      const auto at = std::find(runs_sel_.begin(), runs_sel_.end(), folder);
      const int slot = at == runs_sel_.end() ? -1 : static_cast<int>(at - runs_sel_.begin());
      ImGui::PushID(i);
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      bool on = slot >= 0;
      if (on) ImGui::PushStyleColor(ImGuiCol_CheckMark, kRunColors[slot]);
      if (ImGui::Checkbox("##sel", &on)) RunsToggle(folder);
      if (slot >= 0) ImGui::PopStyleColor();
      ui::RecordTarget(Fmt("runs:sel:%d", i));
      if (slot >= 0) { ImGui::SameLine(0, 2); ImGui::TextColored(kRunColors[slot], "%s", kLetters[slot]); }
      const std::string name = r.value("name", std::string());
      ImGui::TableNextColumn();
      // 20260929_003602_controller -> 09-29 00:36:02
      ImGui::TextUnformatted(name.size() >= 15 ? Fmt("%s-%s %s:%s:%s", name.substr(4, 2).c_str(), name.substr(6, 2).c_str(),
                                                     name.substr(9, 2).c_str(), name.substr(11, 2).c_str(), name.substr(13, 2).c_str()).c_str()
                                               : name.c_str());
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n%s", folder.c_str(), r.value("end_reason", std::string()).c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(r.value("controller", std::string()).c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(r.value("vehicle", std::string()).c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(r.value("map", json("")).is_string() ? r["map"].get<std::string>().c_str() : "");
      ImGui::TableNextColumn();
      const double dur = Num(r, "duration");
      ImGui::TextUnformatted(std::isnan(dur) ? "—" : Fmt("%.1f", dur).c_str());
      const json k = r.value("kpi", json::object());
      ImGui::TableNextColumn();
      const double rms = Num(k, "lane_offset_rms");
      ImGui::TextUnformatted(std::isnan(rms) ? "—" : Fmt("%.3f", rms).c_str());
      ImGui::TableNextColumn();
      const double col = Num(k, "collisions");
      if (!std::isnan(col) && col > 0) ImGui::TextColored(p.danger, "%.0f", col);
      else ImGui::TextUnformatted(std::isnan(col) ? "—" : "0");
      ImGui::PopID();
      ++i;
    }
    ImGui::EndTable();
  }
  if (runs.empty() && err.empty()) ImGui::TextColored(p.text_dim, "还没有运行记录");
  ui::EndCard();

  if (runs_sel_.empty()) return;
  ui::BeginCard(ICON_FA_TABLE, "指标对比");
  struct K { const char* key; const char* name; int better; const char* fmt; };  // better: -1 lower, +1 higher, 0 neither
  static const K kKpis[] = {{"lane_offset_rms", "车道偏差 均方根 (m)", -1, "%.3f"}, {"lane_offset_max", "车道偏差 最大 (m)", -1, "%.3f"},
                            {"heading_err_max", "最大航向偏差", -1, "%.2f"}, {"time_off_lane", "出车道时间 (s)", -1, "%.2f"},
                            {"collisions", "碰撞次数", -1, "%.0f"}, {"min_gap_ahead", "前方最小间距 (m)", +1, "%.2f"},
                            {"ay_max", "最大侧向加速度", -1, "%.2f"}, {"distance", "行驶距离 (m)", 0, "%.0f"}};
  std::vector<json> picked;
  for (const auto& f : runs_sel_)
    for (const json& r : runs)
      if (r.value("folder", std::string()) == f) picked.push_back(r);
  const int n = static_cast<int>(picked.size());
  if (ImGui::BeginTable("##kpis", n + 1, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("指标", ImGuiTableColumnFlags_WidthStretch, 2.2f);
    for (int j = 0; j < n; ++j) ImGui::TableSetupColumn(kLetters[j], ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("指标");
    for (int j = 0; j < n; ++j) {
      ImGui::TableNextColumn();
      ImGui::TextColored(kRunColors[j], "%s  %s", kLetters[j], picked[j].value("controller", std::string()).c_str());
    }
    auto row = [&](const char* name, const std::vector<double>& v, int better, const char* fmt) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(name);
      double best = NAN;
      if (better)
        for (double x : v) if (!std::isnan(x)) best = std::isnan(best) ? x : (better < 0 ? std::min(best, x) : std::max(best, x));
      for (double x : v) {
        ImGui::TableNextColumn();
        if (std::isnan(x)) { ImGui::TextColored(p.text_dim, "—"); continue; }
        const bool win = better && n > 1 && x == best;
        if (win) ImGui::TextColored(p.success, fmt, x);
        else ImGui::Text(fmt, x);
      }
    };
    std::vector<double> dur;
    for (const json& r : picked) dur.push_back(Num(r, "duration"));
    row("时长 (s)", dur, 0, "%.1f");
    for (const K& k : kKpis) {
      std::vector<double> v;
      for (const json& r : picked) v.push_back(Num(r.value("kpi", json::object()), k.key));
      row(k.name, v, k.better, k.fmt);
    }
    ImGui::EndTable();
  }
  ui::RecordTarget("runs:kpi");
  ui::DimWrapped("绿色 = 这一项最好的一次。角度、加速度的单位跟随那次运行的“CarSim 动力学”页设置。");
  if (ui::Button(ICON_FA_CODE_COMPARE, "打开对比图", ui::Kind::Primary)) {
    compare_open_ = true;
    compare_focus_ = true;
  }
  ui::RecordTarget("runs:open");
  ImGui::SameLine();
  ImGui::BeginDisabled(!be_.Connected() || run_out_pending_);
  if (ui::Button(ICON_FA_TERMINAL, "A 的输出")) {
    const std::string folder = runs_sel_[0];
    run_out_pending_ = true;
    be_.Request("run_output", {{"folder", folder}}, [this, folder](bool ok, const json& r, const std::string& err) {
      run_out_pending_ = false;
      run_out_folder_ = folder;
      run_out_ = ok ? r : json{{"error", err}};
      run_out_open_ = true;
    });
  }
  ui::RecordTarget("runs:output");
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("这次运行时“输出”窗口的全部内容（算法的 print、报错、运行指标），存在运行文件夹的 output.txt 里");
  ImGui::EndDisabled();
  ui::EndCard();
  DrawIdent();
}

void App::DrawRunOutput() {
  if (!run_out_open_) return;
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::SetNextWindowSize(ImVec2(fs * 60, fs * 32), ImGuiCond_FirstUseEver);
  if (ImGui::Begin((std::string(ICON_FA_TERMINAL "  运行输出 · ") + fs::u8path(run_out_folder_).filename().u8string() +
                    "###runoutput").c_str(), &run_out_open_)) {
    if (run_out_.contains("error")) {
      ImGui::TextColored(p.warning, "%s", run_out_.value("error", std::string()).c_str());
    } else {
      const std::string text = run_out_.value("text", std::string());
      ImGui::Checkbox("只看算法输出", &run_out_algo_only_);
      ImGui::SameLine();
      if (ui::Button(ICON_FA_COPY, "复制全部")) ImGui::SetClipboardText(text.c_str());
      ImGui::SameLine();
      ImGui::TextColored(p.text_dim, "%d 行%s  ·  %s", run_out_.value("lines", 0),
                         run_out_.value("truncated", false) ? "（太长，只显示最后 2 MB）" : "",
                         (fs::u8path(run_out_folder_) / "output.txt").u8string().c_str());
      ImGui::BeginChild("##outtext", ImVec2(0, 0), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
      size_t a = 0;
      while (a < text.size()) {
        size_t b = text.find('\n', a);
        if (b == std::string::npos) b = text.size();
        const std::string line = text.substr(a, b - a);
        a = b + 1;
        const bool algo = line.find("[算法] ") != std::string::npos;
        if (run_out_algo_only_ && !algo) continue;
        const ImVec4 col = line.find("[错误] ") != std::string::npos ? p.danger
                           : line.find("[警告] ") != std::string::npos ? p.warning
                           : algo ? p.text : p.text_dim;
        ImGui::TextColored(col, "%s", line.c_str());
      }
      ui::RecordTarget("runs:output_text");
      ImGui::EndChild();
    }
  }
  ImGui::End();
}

void App::DrawIdent() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const std::string folder = runs_sel_.empty() ? std::string() : runs_sel_[0];
  ui::BeginCard(ICON_FA_WRENCH, "车辆参数辨识（A）");
  ui::DimWrapped("从 A 这次运行（CarSim、Chrono 宝马或模拟 CarSim）的导出变量辨识 KMPPI 预测模型的前 / 后轴轮胎侧向力比例因子"
                 "（方法同原工程 identify_plant.py，用每个样本的 3DOF 力平衡）。下面填这台车的参数（CarSim 车辆参数里查）；"
                 "要一次有弯道或蛇行的运行（侧向加速度最大至少 1 m/s²）。");
  if (!cfg_.contains("ident") || !cfg_["ident"].is_object()) cfg_["ident"] = json::object();
  json& id = cfg_["ident"];
  struct F { const char* key; const char* name; const char* tip; double def; };
  static const F kFields[] = {{"m", "质量 m (kg)", "整车质量", 1910.0},
                              {"I", "横摆惯量 I (kg·m²)", "绕竖直轴的转动惯量", 3482.0},
                              {"a", "质心到前轴 a (m)", "", 1.371},
                              {"b", "质心到后轴 b (m)", "", 1.386}};
  for (const F& f : kFields) {
    ui::Row(f.name, f.tip, fs * 10);
    double v = id.value(f.key, f.def);
    if (ImGui::InputDouble(Fmt("##ident_%s", f.key).c_str(), &v, 0.0, 0.0, "%g") && v > 0) id[f.key] = v;
    ui::RecordTarget(std::string("ident:") + f.key);
  }
  ui::Row("保存到", "KMPPI 的车辆参数文件（相对路径以 carsim_carla_bridge 目录为准）", fs * 16);
  std::string path = id.value("path", std::string("controllers/kmppi/vehicle_identified.json"));
  if (appui::InputStr("##ident_path", path)) id["path"] = path;
  auto request = [this, folder](const std::string& save) {
    const json& d = cfg_["ident"];
    ident_pending_ = true;
    be_.Request("vehicle_identify",
                {{"folder", folder}, {"m", d.value("m", 1910.0)}, {"I", d.value("I", 3482.0)}, {"a", d.value("a", 1.371)},
                 {"b", d.value("b", 1.386)}, {"save_path", save}},
                [this, folder, save](bool ok, const json& r, const std::string& err) {
                  ident_pending_ = false;
                  ident_folder_ = folder;
                  ident_result_ = ok ? r : json{{"error", err}};
                  if (!ok) return;
                  if (save.empty()) {
                    Log(Fmt("车辆参数辨识：轮胎比例因子 前 %.3f、后 %.3f（R² %.3f，%d 个样本）", r.value("front_lateral_scale", 0.0),
                            r.value("rear_lateral_scale", 0.0), r.value("r2", 0.0), r.value("samples", 0)));
                    return;
                  }
                  // KMPPI's VEHICLE_JSON (the 算法参数 of the current algorithm file) set to the file.
                  const std::string algo = cfg_["run"]["controller"].value("path", std::string());
                  bool has = false;
                  if (algo == algo_params_path_)
                    for (const json& prm : algo_params_.value("params", json::array()))
                      has = has || prm.value("name", std::string()) == "VEHICLE_JSON";
                  const std::string saved = r.value("saved", std::string());
                  if (has) {
                    cfg_["run"]["params"][algo]["VEHICLE_JSON"] = saved;
                    Log("车辆参数已保存到 " + saved + "；“驾驶模式”页的算法参数 VEHICLE_JSON 已设为它，下次运行 KMPPI 就用这组参数");
                  } else {
                    Log("车辆参数已保存到 " + saved + "；当前的算法文件没有 VEHICLE_JSON 参数：用 KMPPI（controllers/kmppi/controller.py）"
                        "时在它的算法参数里把 VEHICLE_JSON 设为这个文件", "warn");
                  }
                });
  };
  const bool can = !folder.empty() && be_.Connected() && !ident_pending_;
  ImGui::BeginDisabled(!can);
  if (ui::Button(ICON_FA_WAND_MAGIC_SPARKLES, ident_pending_ ? "辨识中…" : "辨识")) request("");
  ui::RecordTarget("ident:run");
  ImGui::EndDisabled();
  const json& r = ident_result_;
  if (ident_folder_ != folder || r.is_null()) {
    ui::EndCard();
    return;
  }
  if (r.contains("error")) {
    ImGui::PushTextWrapPos(0);
    ImGui::TextColored(p.warning, "%s", r.value("error", std::string()).c_str());
    ImGui::PopTextWrapPos();
    ui::RecordTarget("ident:result");
    ui::EndCard();
    return;
  }
  if (ImGui::BeginTable("##ident", 2, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
    auto row = [&](const char* name, const std::string& v) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(name);
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(v.c_str());
    };
    row("前轴轮胎比例因子", Fmt("%.4f", r.value("front_lateral_scale", 0.0)));
    row("后轴轮胎比例因子", Fmt("%.4f", r.value("rear_lateral_scale", 0.0)));
    row("拟合 R²", Fmt("%.3f", r.value("r2", 0.0)));
    row("用到的样本", Fmt("%d", r.value("samples", 0)));
    row("最大侧向加速度", Fmt("%.2f m/s²", r.value("ay_max", 0.0)));
    row("最大侧偏角 前 / 后", Fmt("%.2f° / %.2f°", r.value("alpha_f_max", 0.0) * 57.29578, r.value("alpha_r_max", 0.0) * 57.29578));
    ImGui::EndTable();
  }
  ui::RecordTarget("ident:result");
  if (r.value("r2", 0.0) < 0.8)
    ui::DimWrapped("R² 偏低：这次运行的转向可能太少或太剧烈（超出线性区），也可能车辆参数填得不对。");
  if (r.contains("saved")) ImGui::TextColored(p.success, "已保存：%s", r.value("saved", std::string()).c_str());
  ImGui::BeginDisabled(!can);
  if (ui::Button(ICON_FA_FLOPPY_DISK, "保存并用于 KMPPI", ui::Kind::Primary)) request(id.value("path", std::string()));
  ui::RecordTarget("ident:save");
  ImGui::EndDisabled();
  ui::EndCard();
}

void App::DrawCompare() {
  const ui::Palette& p = ui::Colors();
  std::vector<std::pair<int, const json*>> ser;  // (slot, series)
  for (size_t j = 0; j < runs_sel_.size(); ++j) {
    auto it = runs_series_.find(runs_sel_[j]);
    if (it != runs_series_.end()) ser.push_back({static_cast<int>(j), &it->second});
  }
  if (ser.empty()) {
    ImGui::TextColored(p.text_dim, runs_pending_ ? "正在读取运行记录 ..." : "在“运行对比”页（工程 → 数据）勾选要比较的运行");
    return;
  }
  // Legend and the self.debug value to plot (the names of the picked runs).
  for (const auto& [slot, s] : ser) {
    ImGui::TextColored(kRunColors[slot], "■ %s  %s", kLetters[slot], s->value("name", std::string()).c_str());
    ImGui::SameLine(0, ImGui::GetFontSize());
  }
  std::vector<std::string> dbg;
  for (const auto& [slot, s] : ser) {
    const json d = s->value("debug", json::object());  // (not items() of a temporary: it would be gone)
    for (const auto& kv : d.items())
      if (std::find(dbg.begin(), dbg.end(), kv.key()) == dbg.end()) dbg.push_back(kv.key());
  }
  if (!dbg.empty()) {
    if (std::find(dbg.begin(), dbg.end(), compare_dbg_) == dbg.end()) compare_dbg_ = dbg[0];
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    if (ImGui::BeginCombo("##cmpdbg", compare_dbg_.c_str())) {
      for (const auto& n : dbg)
        if (ImGui::Selectable(n.c_str(), n == compare_dbg_)) compare_dbg_ = n;
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextColored(p.text_dim, "算法的 self.debug");
  } else {
    ImGui::NewLine();
  }
  // Timeline: play / pause, the time, the speed; 在 CARLA 里看这一刻 (run A).
  double t_max = 0.0;
  for (const auto& [slot, s] : ser) {
    const json tj = s->value("t", json::array());
    if (!tj.empty() && tj.back().is_number()) t_max = std::max(t_max, tj.back().get<double>());
  }
  if (compare_play_) {
    compare_t_ += ImGui::GetIO().DeltaTime * compare_speed_;
    if (compare_t_ >= t_max) { compare_t_ = t_max; compare_play_ = false; }
  }
  compare_t_ = std::clamp(compare_t_, 0.0, t_max);
  if (ui::IconButton(compare_play_ ? ICON_FA_PAUSE : ICON_FA_PLAY, compare_play_ ? "暂停" : "播放", "cmpplay")) {
    if (!compare_play_ && compare_t_ >= t_max) compare_t_ = 0.0;
    compare_play_ = !compare_play_;
  }
  ui::RecordTarget("compare:play");
  ImGui::SameLine();
  float tf = static_cast<float>(compare_t_);
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 18);
  if (ImGui::SliderFloat("##cmpt", &tf, 0.0f, static_cast<float>(t_max), "t = %.2f s")) { compare_t_ = tf; compare_play_ = false; }
  ui::RecordTarget("compare:time");
  ImGui::SameLine();
  static const float kSpeeds[] = {0.5f, 1.0f, 2.0f, 4.0f};
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4.5f);
  if (ImGui::BeginCombo("##cmpspeed", Fmt("×%g", compare_speed_).c_str())) {
    for (float sp : kSpeeds)
      if (ImGui::Selectable(Fmt("×%g", sp).c_str(), sp == compare_speed_)) compare_speed_ = sp;
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(Running() || !carla_connected_);
  ImGui::Checkbox("在 CARLA 里看这一刻（A）", &compare_carla_);
  ImGui::EndDisabled();
  ui::RecordTarget("compare:carla");
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("把主车摆到 A 这次运行在这一刻记录的位置和航向（地图要是那次运行的地图），画面和视角都跟着；下一次运行会重新生成主车");
  if (compare_carla_ && (Running() || !carla_connected_)) compare_carla_ = false;
  if (compare_carla_ && !compare_replay_pending_ && std::fabs(compare_t_ - compare_sent_t_) > 1e-3 &&
      ImGui::GetTime() - compare_sent_at_ > 0.08) {
    compare_replay_pending_ = true;
    compare_sent_t_ = compare_t_;
    compare_sent_at_ = ImGui::GetTime();
    be_.Request("replay_pose", {{"folder", ser[0].second->value("folder", std::string())}, {"t", compare_t_}},
                [this](bool ok, const json&, const std::string& err) {
                  compare_replay_pending_ = false;
                  if (!ok) {
                    Log("在 CARLA 里回放：" + err, "warn");
                    compare_carla_ = false;
                    compare_sent_t_ = -1.0;
                  }
                });
  }
  if (!compare_carla_) compare_sent_t_ = -1.0;  // switched on again: send at once
  // The values at this time, a line per run.
  for (const auto& [slot, s] : ser) {
    const std::vector<float> t = Floats(s->value("t", json::array()));
    const json u = s->value("u", json::array()), names = s->value("u_names", json::array());
    std::string line = Fmt("%s  车速 %.1f  偏差 %.3f m", kLetters[slot], ValueAt(t, Floats(s->value("speed", json::array())), compare_t_),
                           ValueAt(t, Floats(s->value("offset", json::array())), compare_t_));
    for (size_t i = 0; i < u.size() && i < names.size(); ++i)
      line += Fmt("  %s %.3g", names[i].get<std::string>().c_str(), ValueAt(t, Floats(u[i]), compare_t_));
    const json d = s->value("debug", json::object());
    if (!compare_dbg_.empty() && d.contains(compare_dbg_))
      line += Fmt("  %s %.4g", compare_dbg_.c_str(), ValueAt(t, Floats(d[compare_dbg_]), compare_t_));
    ImGui::TextColored(kRunColors[slot], "%s", line.c_str());
  }
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float gap = ImGui::GetStyle().ItemSpacing.x;
  const ImVec2 sz((avail.x - gap * 2) / 3.0f, (avail.y - ImGui::GetStyle().ItemSpacing.y) * 0.5f);
  const ImPlotFlags pf = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect | ImPlotFlags_NoLegend;
  ImPlotSpec line;
  line.LineWeight = 1.6f;
  auto series_plot = [&](const char* title, const char* id, auto&& get) {
    if (ImPlot::BeginPlot(Fmt("%s###%s", title, id).c_str(), sz, pf)) {
      ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
      for (const auto& [slot, s] : ser) {
        const std::vector<float> t = Floats(s->value("t", json::array())), y = Floats(get(*s));
        const int nn = static_cast<int>(std::min(t.size(), y.size()));
        ImPlotSpec spec = line;
        spec.LineColor = kRunColors[slot];
        if (nn > 1) ImPlot::PlotLine(kLetters[slot], t.data(), y.data(), nn, spec);
      }
      ImPlotSpec cur;
      cur.LineColor = ImVec4(1, 1, 1, 0.55f);
      ImPlot::PlotInfLines("##t", &compare_t_, 1, cur);  // the timeline's time
      ImPlot::EndPlot();
    }
  };
  // Row 1: the trajectories (CarSim global frame, x forward at the start, y left), speed, lane offset.
  if (ImPlot::BeginPlot("轨迹（CarSim 全局坐标，m）###cmpxy", sz, pf | ImPlotFlags_Equal)) {
    ImPlot::SetupAxes("X", "Y", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
    for (const auto& [slot, s] : ser) {
      const std::vector<float> x = Floats(s->value("x", json::array())), y = Floats(s->value("y", json::array()));
      ImPlotSpec spec = line;
      spec.LineColor = kRunColors[slot];
      const int nn = static_cast<int>(std::min(x.size(), y.size()));
      if (nn > 1) ImPlot::PlotLine(kLetters[slot], x.data(), y.data(), nn, spec);
      // Where it is at the timeline's time.
      const std::vector<float> t = Floats(s->value("t", json::array()));
      const double px = ValueAt(t, x, compare_t_), py = ValueAt(t, y, compare_t_);
      if (std::isfinite(px) && std::isfinite(py)) {
        ImPlotSpec dot;
        dot.Marker = ImPlotMarker_Circle;
        dot.MarkerSize = 6;
        dot.MarkerFillColor = kRunColors[slot];
        dot.MarkerLineColor = ImVec4(1, 1, 1, 1);
        ImPlot::PlotScatter("##pos", &px, &py, 1, dot);
      }
    }
    ImPlot::EndPlot();
  }
  ui::RecordTarget("compare:plots");
  ImGui::SameLine();
  series_plot("车速", "cmpv", [](const json& s) { return s.value("speed", json::array()); });
  ImGui::SameLine();
  series_plot("车道偏差 (m)", "cmpoff", [](const json& s) { return s.value("offset", json::array()); });
  // Row 2: the algorithm's outputs (named by the first run's kind), its self.debug value.
  const json names = ser[0].second->value("u_names", json::array());
  auto uname = [&](size_t i) { return names.size() > i && names[i].is_string() ? names[i].get<std::string>() : Fmt("导入 %d", static_cast<int>(i + 1)); };
  const size_t last_u = names.size() >= 3 ? 2 : 1;
  series_plot(uname(0).c_str(), "cmpu0", [](const json& s) { const json u = s.value("u", json::array()); return u.size() > 0 ? u[0] : json::array(); });
  ImGui::SameLine();
  series_plot(uname(last_u).c_str(), "cmpu1", [last_u](const json& s) { const json u = s.value("u", json::array()); return u.size() > last_u ? u[last_u] : json::array(); });
  ImGui::SameLine();
  if (!dbg.empty())
    series_plot(Fmt("%s（算法）", compare_dbg_.c_str()).c_str(), "cmpdbg",
                [this](const json& s) { const json d = s.value("debug", json::object()); return d.contains(compare_dbg_) ? d[compare_dbg_] : json::array(); });
  else
    series_plot("航向偏差", "cmphe", [](const json& s) { return s.value("heading_err", json::array()); });
}
