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
    if (!carla_connected_ && !batch_stop_) {
      // CARLA went away in this item: waited for below, then the item again (once).
      r["detail"] = "CARLA 断开：" + r.value("detail", std::string());
      const std::string key = std::to_string(batch_i_);
      if (!batch_retried_.count(key)) {
        batch_retried_.insert(key);
        --batch_i_;
        Log("批量测试：CARLA 断开了。等 CARLA 重新启动后自动重连，重跑这一项再继续（没有别的程序会重启 CARLA 时，请手动启动它）", "warn");
      }
      batch_lost_ = true;
      batch_ready_since_ = -1.0;
    }
    // The report so far (after every run: a batch stopped half way has one too).
    Call("batch_summary", {{"dir", batch_dir_}, {"items", batch_results_}}, [this](const json& rep) { batch_report_ = rep; });
  }
  if (batch_stop_ || batch_i_ + 1 >= static_cast<int>(batch_items_.size())) {
    batch_running_ = false;
    Log(Fmt("批量测试%s：%d 项运行完，报告在 %s（report.md / report.csv）", batch_stop_ ? "已停止" : "完成",
            static_cast<int>(batch_results_.size()), batch_dir_.c_str()));
    return;
  }
  if (batch_lost_ && !batch_stop_) {
    if (!carla_connected_) {
      // Is CARLA's port listening again? (the kernel's table: never by connecting, see
      // carla_port_ready). Listening for 20 s (the map loads): connect once; every 30 s at most.
      const double now = ImGui::GetTime();
      if (!busy_.empty() || batch_probe_pending_ || now - batch_probe_at_ < 5.0) return;
      batch_probe_at_ = now;
      batch_probe_pending_ = true;
      be_.Request("carla_port_ready", {{"host", prefs_.value("carla_host", std::string("localhost"))}, {"port", prefs_.value("carla_port", 2000)}},
                  [this](bool ok, const json& r, const std::string&) {
                    batch_probe_pending_ = false;
                    const json l = ok ? r.value("listening", json()) : json(false);
                    const double t = ImGui::GetTime();
                    if (l.is_boolean() && !l.get<bool>()) { batch_ready_since_ = -1.0; return; }
                    if (l.is_boolean()) {
                      if (batch_ready_since_ < 0) batch_ready_since_ = t;
                      if (t - batch_ready_since_ < 20.0) return;
                    }
                    if (t - batch_connect_at_ < 30.0 || !batch_running_) return;
                    batch_connect_at_ = t;
                    Log("批量测试：CARLA 又在监听了，重新连接");
                    ConnectCarla();
                  });
      return;
    }
    // Back: the batch's map again (a restarted CARLA starts on its default map).
    const std::string map = world_.value("map", std::string());
    if (!batch_map_.empty() && !map.empty() && map != batch_map_) {
      if (busy_.empty()) LoadMap(batch_map_);
      return;
    }
    if (map.empty()) return;
    batch_lost_ = false;
    Log("批量测试：CARLA 已重新连接，继续");
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

void App::DrawCriteria() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  if (!cfg_.contains("criteria") || !cfg_["criteria"].is_object()) cfg_["criteria"] = json::object();
  json& c = cfg_["criteria"];
  if (!c.contains("limits") || !c["limits"].is_object()) c["limits"] = json::object();
  ui::BeginCard(ICON_FA_SCALE_BALANCED, "通过标准（每次运行结束时判定）");
  ImGui::BeginDisabled(Running() || batch_running_);
  ui::DimWrapped("每次运行结束时按这些标准判定通过 / 未通过：输出窗口说明原因，run.json 记下（verdict），批量测试的报告和“运行对比”页都用它。随工程保存。");
  struct B { const char* key; const char* name; };
  static const B kBasic[] = {{"finished", "要跑完设定的时长"}, {"no_collision", "不能有碰撞"}, {"on_lane", "不能开出行车道"}};
  for (const B& b : kBasic) {
    bool v = c.value(b.key, true);
    if (ImGui::Checkbox(b.name, &v)) c[b.key] = v;
    ImGui::SameLine();
  }
  ImGui::NewLine();
  struct L { const char* key; const char* name; const char* op; const char* unit; double def; double step; };
  static const L kLimits[] = {{"lane_offset_rms", "车道偏差均方根", "≤", "m", 0.3, 0.05},
                              {"lane_offset_max", "车道偏差最大", "≤", "m", 0.8, 0.05},
                              {"ttc_min", "最小碰撞时间 TTC", "≥", "s", 2.0, 0.5},
                              {"min_gap_ahead", "前方最小间距", "≥", "m", 5.0, 1.0},
                              {"accel_max", "最大加速度", "≤", "m/s²", 3.0, 0.5},
                              {"decel_max", "最大减速度", "≤", "m/s²", 6.0, 0.5},
                              {"jerk_max", "最大纵向 jerk", "≤", "m/s³", 10.0, 1.0}};
  json& lim = c["limits"];
  for (const L& l : kLimits) {
    ImGui::PushID(l.key);
    bool on = lim.contains(l.key) && lim[l.key].is_number();
    if (ImGui::Checkbox("##on", &on)) lim[l.key] = on ? json(l.def) : json();
    ui::RecordTarget(std::string("crit:") + l.key);
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    if (on) ImGui::TextUnformatted(Fmt("%s %s", l.name, l.op).c_str());
    else ImGui::TextColored(p.text_dim, "%s（不检查）", l.name);
    if (on) {
      ImGui::SameLine(fs * 12.5f);
      double v = lim[l.key].get<double>();
      ImGui::SetNextItemWidth(fs * 7);
      if (ImGui::InputDouble("##v", &v, l.step, l.step * 10, "%g")) lim[l.key] = std::max(0.0, v);
      ImGui::SameLine();
      ImGui::TextColored(p.text_dim, "%s", l.unit);
    }
    ImGui::PopID();
  }
  ui::DimWrapped("没有数据的一项不算违反（例如前方一直没有接近的目标，TTC 为空）。加速度、jerk 按采样周期（“数据采集”页，默认 0.1 s）由车速差分。");
  ImGui::EndDisabled();
  ui::EndCard();
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
  ui::Row("关闭渲染（更快）", "运行时 CARLA 不渲染画面，只算物理：约快 2.5 倍（实测路线跟随 30 s：16 s → 6.5 s）。运行中实时画面不更新；要采集数据或算法要用相机传感器时自动不关（输出窗口会说明）。运行结束后恢复原来的设置（和“驾驶模式”页的“运行时关闭渲染”是同一个设置）");
  bool nr = cfg_["sync"].value("no_render", false);
  if (ImGui::Checkbox("##batch_norender", &nr)) cfg_["sync"]["no_render"] = nr;
  ui::RecordTarget("batch:norender");
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
      batch_lost_ = false;
      batch_retried_.clear();
      batch_map_ = world_.value("map", std::string());
    }
    ui::RecordTarget("batch:start");
    ImGui::EndDisabled();
  } else {
    if (batch_lost_) {
      ImGui::PushTextWrapPos(0.0f);
      ImGui::TextColored(p.warning, ICON_FA_TRIANGLE_EXCLAMATION "  %s",
                         carla_connected_ ? ("CARLA 已重新连接，正在切回地图 " + batch_map_ + " …").c_str()
                                          : "CARLA 断开了：等它重新启动（端口重新监听 20 s 后自动重连），然后重跑断开的那一项、继续往下跑。"
                                            "没有别的程序会重启 CARLA 时，请手动启动它。");
      ImGui::PopTextWrapPos();
      ui::RecordTarget("batch:lost");
    }
    // (waiting for CARLA: the item that broke, run again next)
    const int n_items = static_cast<int>(batch_items_.size()), shown = std::min(batch_i_ + 1 + (batch_lost_ ? 1 : 0), n_items);
    ImGui::ProgressBar(static_cast<float>(shown) / std::max(1, n_items), ImVec2(-FLT_MIN, 0), Fmt("第 %d / %d 项", shown, n_items).c_str());
    if (ui::Button(ICON_FA_STOP, batch_stop_ ? "正在停止 ..." : "停止批量测试", ui::Kind::Danger) && !batch_stop_) {
      batch_stop_ = true;
      if (Running()) RunCommand("cosim_stop");
    }
    ui::RecordTarget("batch:stop");
  }
  ui::EndCard();
  DrawCriteria();

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
      std::string d = row.is_object() ? row.value("detail", std::string()) : res.value("detail", std::string());
      if (row.is_object())  // why it did not pass (通过标准), then how it ended
        for (const json& f : row.value("fails", json::array())) d = f.get<std::string>() + "；" + d;
      ImGui::TextUnformatted(d.c_str());
      if (ImGui::IsItemHovered() && !d.empty()) ImGui::SetTooltip("%s", d.c_str());
    }
    ImGui::EndTable();
  }
  if (batch_report_.contains("md")) {
    ImGui::TextColored(p.text_dim, "通过 %d / %d 项（按上面的“通过标准”；未通过的原因在“说明”里）", batch_report_.value("passed", 0),
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
