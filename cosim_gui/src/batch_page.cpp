// 批量测试: scenario presets x spawn points x values of one algorithm constant,
// run one after the other (each a normal run: StartRun with this item's
// settings merged over the page's config, its record in runs/batch_<time>/),
// the results and a report (the backend's batch_summary: report.csv,
// report.md in the batch's folder), refreshed after every run.
#include <algorithm>
#include <cmath>
#include <ctime>
#include <sstream>

#include "app.h"
#include "ui_kit.h"

using appui::Fmt;

namespace {
std::vector<std::string> SplitList(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s + ",") {
    if (c == ',' || c == ';' || c == '\n') {
      const size_t a = cur.find_first_not_of(" \t"), b = cur.find_last_not_of(" \t");
      if (a != std::string::npos) out.push_back(cur.substr(a, b - a + 1));
      cur.clear();
    } else {
      cur += c;
    }
  }
  return out;
}
}  // namespace

std::vector<json> App::BatchPlan() {
  std::vector<std::pair<std::string, json>> scenes;
  if (batch_scn_[0]) scenes.push_back({"不开封道", {{"enabled", false}, {"closures", json::array()}, {"actors", json::array()}}});
  const auto& presets = TestScenePresets();
  for (size_t i = 0; i < presets.size() && i < 4; ++i)
    if (batch_scn_[i + 1])
      scenes.push_back({presets[i].name, {{"enabled", true}, {"closures", presets[i].closures}, {"actors", json::array()}}});
  const auto& actor_presets = TestActorPresets();
  for (size_t i = 0; i < actor_presets.size() && i < 4; ++i)
    if (batch_scn_[i + 5])
      scenes.push_back({actor_presets[i].name, {{"enabled", true}, {"closures", json::array()}, {"actors", actor_presets[i].closures}}});
  std::vector<int> spawns;
  for (const auto& t : SplitList(batch_spawns_)) {
    char* end = nullptr;
    const long v = std::strtol(t.c_str(), &end, 10);
    if (end && *end == '\0' && v >= 0) spawns.push_back(static_cast<int>(v));
  }
  if (spawns.empty()) spawns.push_back(cfg_.contains("carla") ? cfg_["carla"].value("spawn_index", 0) : 0);
  // Values of one algorithm constant (算法参数), as the file's type.
  std::vector<json> values;
  std::string type;
  for (const json& p : algo_params_.value("params", json::array()))
    if (p.value("name", std::string()) == batch_param_) type = p.value("type", std::string());
  if (!batch_param_.empty() && !type.empty())
    for (const auto& t : SplitList(batch_values_)) {
      if (type == "str") values.push_back(t);
      else if (type == "bool") values.push_back(t == "1" || t == "True" || t == "true");
      else {
        char* end = nullptr;
        const double v = std::strtod(t.c_str(), &end);
        if (end && *end == '\0') values.push_back(type == "int" ? json(static_cast<int>(std::lround(v))) : json(v));
      }
    }
  const std::string path = cfg_["run"]["controller"].value("path", std::string());
  const json base_params = cfg_["run"].value("params", json::object()).value(path, json::object());
  std::vector<json> items;
  for (const auto& [scene_name, scenario] : scenes)
    for (int sp : spawns) {
      const size_t n_v = values.empty() ? 1 : values.size();
      for (size_t k = 0; k < n_v; ++k) {
        json over = {{"scenario", scenario}, {"carla", {{"spawn_index", sp}}}, {"sync", {{"duration", batch_duration_}}}};
        std::string label = Fmt("%s · 出生点 %d", scene_name.c_str(), sp);
        if (!values.empty()) {
          json prm = base_params.is_object() ? base_params : json::object();
          prm[batch_param_] = values[k];
          over["run"]["params"][path] = prm;
          label += " · " + batch_param_ + " = " + values[k].dump();
        }
        items.push_back({{"label", label}, {"over", over}});
      }
    }
  return items;
}

void App::BatchTick() {
  if (!batch_running_) return;
  if (batch_wait_) {
    if (starting_ || Running()) return;  // this item's run: starting or running
    json& r = batch_results_.back();
    if (r.value("started", false)) {
      r["state"] = run_state_;
      r["detail"] = run_note_;
    }
    batch_wait_ = false;
    // The report so far (after every run: a batch stopped half way has one too).
    Call("batch_summary", {{"dir", batch_dir_}, {"items", batch_results_}}, [this](const json& rep) { batch_report_ = rep; });
  }
  if (batch_stop_ || batch_i_ + 1 >= static_cast<int>(batch_items_.size())) {
    batch_running_ = false;
    Log(Fmt("批量测试%s：%d 项运行完，报告在 %s（report.md / report.csv）", batch_stop_ ? "已停止" : "完成",
            static_cast<int>(batch_results_.size()), batch_dir_.c_str()));
    return;
  }
  if (!busy_.empty() || !carla_connected_ || be_.PendingCount() > 0) return;
  ++batch_i_;
  const json item = batch_items_[static_cast<size_t>(batch_i_)];
  json over = item["over"];
  over["run"]["log_path"] = batch_dir_;
  batch_results_.push_back({{"label", item["label"]}, {"record_dir", ""}, {"state", "未启动"}, {"detail", ""}, {"started", false}});
  batch_wait_ = true;
  Log(Fmt("批量测试 %d / %d：%s", batch_i_ + 1, static_cast<int>(batch_items_.size()), item.value("label", std::string()).c_str()));
  const size_t slot = batch_results_.size() - 1;
  StartRun(over, [this, slot](bool ok, const json& r, const std::string& err) {
    if (slot >= batch_results_.size()) return;
    json& res = batch_results_[slot];
    res["started"] = ok;
    if (ok) res["record_dir"] = r.value("record_dir", std::string());
    else res["detail"] = err;
  });
}

void App::DrawPanelBatch() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const std::vector<json> plan = BatchPlan();
  ui::BeginCard(ICON_FA_LIST_CHECK, "测试项");
  ImGui::BeginDisabled(batch_running_);
  ui::SectionCaption("场景（“测试场景”页的预设：施工封道、动态目标）");
  const auto& presets = TestScenePresets();
  const auto& actor_presets = TestActorPresets();
  for (int i = 0; i < 9; ++i) {
    const char* name = i == 0 ? "不开封道" : i < 5 ? presets[static_cast<size_t>(i - 1)].name : actor_presets[static_cast<size_t>(i - 5)].name;
    // Side by side while they fit, else on the next line.
    const float w = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(name).x;
    if (i && ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + w < ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x)
      ImGui::SameLine();
    ImGui::Checkbox(name, &batch_scn_[i]);
    ui::RecordTarget(Fmt("batch:scn:%d", i));
    if (i && ImGui::IsItemHovered())
      ImGui::SetTooltip("%s", i < 5 ? presets[static_cast<size_t>(i - 1)].tip : actor_presets[static_cast<size_t>(i - 5)].tip);
  }
  ui::Row("出生点", "留空 = 当前的出生点；几个出生点用逗号分开，例如 41, 3, 10", fs * 8);
  char buf[256];
  std::snprintf(buf, sizeof(buf), "%s", batch_spawns_.c_str());
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (ImGui::InputTextWithHint("##bspawns", Fmt("当前：%d", cfg_.contains("carla") ? cfg_["carla"].value("spawn_index", 0) : 0).c_str(),
                               buf, sizeof(buf)))
    batch_spawns_ = buf;
  ui::Row("参数扫描", "选一个算法参数（“驾驶模式”页的算法参数），每个值各跑一遍", fs * 8);
  const json prms = algo_params_.value("params", json::array());
  ImGui::SetNextItemWidth(fs * 9);
  if (ImGui::BeginCombo("##bparam", batch_param_.empty() ? "（不扫描）" : batch_param_.c_str())) {
    if (ImGui::Selectable("（不扫描）", batch_param_.empty())) batch_param_.clear();
    for (const json& prm : prms) {
      const std::string n = prm.value("name", std::string());
      if (ImGui::Selectable(n.c_str(), n == batch_param_)) batch_param_ = n;
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s（文件里 %s）", prm.value("comment", std::string()).c_str(), prm.value("value", json()).dump().c_str());
    }
    ImGui::EndCombo();
  }
  if (!batch_param_.empty()) {
    ImGui::SameLine();
    std::snprintf(buf, sizeof(buf), "%s", batch_values_.c_str());
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::InputTextWithHint("##bvalues", "几个值，逗号分开，例如 15, 20, 25", buf, sizeof(buf))) batch_values_ = buf;
  }
  ui::Row("每次时长 s", "每一项运行多久（仿真时间）", fs * 8);
  ImGui::SetNextItemWidth(fs * 8);
  if (ImGui::InputDouble("##bdur", &batch_duration_, 5.0, 10.0, "%.0f")) batch_duration_ = std::clamp(batch_duration_, 1.0, 3600.0);
  ImGui::EndDisabled();
  ImGui::TextColored(p.text_dim, "共 %d 项（场景 × 出生点 × 参数值），每项 %.0f s；算法、车辆、仿真设置用各页现在的设置", static_cast<int>(plan.size()),
                     batch_duration_);
  if (!batch_running_) {
    ImGui::BeginDisabled(plan.empty() || !carla_connected_ || Running() || !busy_.empty());
    if (ui::Button(ICON_FA_PLAY, Fmt("开始批量测试（%d 项）", static_cast<int>(plan.size())).c_str(), ui::Kind::Primary)) {
      batch_items_ = plan;
      batch_results_.clear();
      batch_report_ = json::object();
      batch_i_ = -1;
      batch_stop_ = batch_wait_ = false;
      std::string base = cfg_["run"].value("log_path", std::string("runs"));
      if (base.empty() || base.size() > 4 && base.substr(base.size() - 4) == ".csv") base = "runs";
      const std::time_t now = std::time(nullptr);
      char ts[32];
      std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
      batch_dir_ = base + "/batch_" + ts;
      batch_running_ = true;
    }
    ui::RecordTarget("batch:start");
    ImGui::EndDisabled();
  } else {
    ImGui::ProgressBar(static_cast<float>(batch_i_ + 1) / std::max<size_t>(1, batch_items_.size()), ImVec2(-FLT_MIN, 0),
                       Fmt("第 %d / %d 项", batch_i_ + 1, static_cast<int>(batch_items_.size())).c_str());
    if (ui::Button(ICON_FA_STOP, batch_stop_ ? "正在停止 ..." : "停止批量测试", ui::Kind::Danger) && !batch_stop_) {
      batch_stop_ = true;
      if (Running()) RunCommand("cosim_stop");
    }
    ui::RecordTarget("batch:stop");
  }
  ui::EndCard();

  if (batch_results_.empty()) return;
  ui::BeginCard(ICON_FA_TABLE, "结果");
  const json rows = batch_report_.value("rows", json::array());
  // A height for its rows: with ScrollX a table takes all the height left in the card and pushes the report out.
  const float rows_h = ImGui::GetTextLineHeightWithSpacing() * (std::min<size_t>(batch_results_.size(), 12) + 1.6f) +
                       ImGui::GetStyle().ScrollbarSize;
  if (ImGui::BeginTable("##bres", 7, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX |
                                         ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit, ImVec2(0, rows_h))) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, fs * 1.6f);
    ImGui::TableSetupColumn("测试项", ImGuiTableColumnFlags_WidthFixed, fs * 14.0f);
    ImGui::TableSetupColumn("结果", ImGuiTableColumnFlags_WidthFixed, fs * 4.0f);
    ImGui::TableSetupColumn("偏差 m", ImGuiTableColumnFlags_WidthFixed, fs * 3.5f);
    ImGui::TableSetupColumn("碰撞", ImGuiTableColumnFlags_WidthFixed, fs * 2.5f);
    ImGui::TableSetupColumn("出车道 s", ImGuiTableColumnFlags_WidthFixed, fs * 3.5f);
    ImGui::TableSetupColumn("说明", ImGuiTableColumnFlags_WidthFixed, fs * 16.0f);
    ImGui::TableHeadersRow();
    for (size_t i = 0; i < batch_results_.size(); ++i) {
      const json& res = batch_results_[i];
      const json row = i < rows.size() ? rows[i] : json::object();
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::Text("%d", static_cast<int>(i + 1));
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(res.value("label", std::string()).c_str());
      ImGui::TableNextColumn();
      const bool running = batch_running_ && static_cast<int>(i) == batch_i_ && batch_wait_;
      if (running) ImGui::TextColored(p.accent, "运行中");
      else if (row.is_object() && row.contains("passed")) {
        if (row.value("passed", false)) ImGui::TextColored(p.success, "通过");
        else ImGui::TextColored(p.danger, "未通过");
      } else ImGui::TextColored(p.text_dim, "…");
      auto num = [&](const char* k, const char* f) {
        ImGui::TableNextColumn();
        if (row.contains(k) && row[k].is_number()) ImGui::Text(f, row[k].get<double>());
        else ImGui::TextColored(p.text_dim, "—");
      };
      num("lane_offset_rms", "%.3f");
      num("collisions", "%.0f");
      num("time_off_lane", "%.2f");
      ImGui::TableNextColumn();
      const std::string d = row.is_object() ? row.value("detail", std::string()) : res.value("detail", std::string());
      ImGui::TextUnformatted(d.c_str());
      if (ImGui::IsItemHovered() && !d.empty()) ImGui::SetTooltip("%s", d.c_str());
    }
    ImGui::EndTable();
  }
  if (batch_report_.contains("md")) {
    ImGui::TextColored(p.text_dim, "通过 %d / %d 项（通过 = 跑完设定的时长、没有碰撞、没有开出车道）", batch_report_.value("passed", 0),
                       batch_report_.value("total", 0));
    ui::DimWrapped(("报告：" + batch_report_.value("md", std::string()) + "（同一个文件夹里还有 report.csv）").c_str());
    ui::RecordTarget("batch:report");
    if (ui::Button(ICON_FA_CODE_COMPARE, "在“运行对比”页打开这批运行")) {
      runs_path_ = batch_report_.value("dir", batch_dir_);
      runs_list_ = json::object();
      runs_sel_.clear();
      panel_ = kPanelRuns;
      RunsRefresh();
    }
    ui::RecordTarget("batch:open_runs");
  }
  ui::EndCard();
}
