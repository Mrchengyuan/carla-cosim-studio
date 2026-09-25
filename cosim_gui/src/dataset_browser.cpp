// Dataset browser: frame-by-frame view of collected sessions with the
// ground-truth boxes, playback, KITTI / nuScenes export and deletion.
// While this page is selected the centre viewport shows the dataset.
#include <algorithm>
#include <cmath>

#include "app.h"
#include "imgui.h"
#include "ui_kit.h"

using appui::ComboStr;
using appui::Fmt;
using appui::InputStr;

namespace {
const ImVec4 kHudBg(0.05f, 0.06f, 0.07f, 0.62f);
const ImVec4 kHudBorder(1, 1, 1, 0.10f);
const ImVec4 kHudText(0.92f, 0.93f, 0.95f, 1);
const ImVec4 kHudDim(0.92f, 0.93f, 0.95f, 0.55f);

const char* TypeName(const std::string& t) {
  return t == "rgb" ? "相机" : t == "semantic" ? "语义" : t == "depth" ? "深度" : t == "instance" ? "实例"
       : t == "lidar" ? "激光雷达" : t == "radar" ? "毫米波雷达" : t.c_str();
}
bool Viewable(const std::string& t) {
  return t == "rgb" || t == "semantic" || t == "depth" || t == "instance" || t == "lidar" || t == "radar";
}
}  // namespace

// --------------------------------------------------------------------------
void App::DatasetRefresh() {
  const std::string dir = cfg_.contains("collect") ? cfg_["collect"].value("out_dir", std::string("datasets")) : "datasets";
  Call("dataset_list", {{"out_dir", dir}}, [this](const json& r) { ds_sessions_ = r; });
}

void App::DatasetOpen(const std::string& root) {
  Call("dataset_info", {{"root", root}}, [this, root](const json& r) {
    ds_info_ = r;
    ds_root_ = root;
    ds_idx_ = 0;
    ds_play_ = false;
    ds_objects_ = json::array();
    for (auto& p : ds_panes_) { p.frames = 0; p.w = p.h = 0; }
    // Default panes: first camera on the left, first lidar (or radar) on the right.
    ds_left_.clear();
    ds_right_.clear();
    for (const json& s : r["sensors"]) {
      const std::string t = s.value("type", std::string()), n = s.value("name", std::string());
      if (ds_left_.empty() && t == "rgb") ds_left_ = n;
      if (ds_right_.empty() && t == "lidar") ds_right_ = n;
    }
    for (const json& s : r["sensors"]) {
      const std::string t = s.value("type", std::string()), n = s.value("name", std::string());
      if (ds_left_.empty() && Viewable(t)) ds_left_ = n;
      if (ds_right_.empty() && Viewable(t) && n != ds_left_) ds_right_ = n;
    }
    ds_export_cam_ = ds_left_;
    ds_export_lidar_.clear();
    for (const json& s : r["sensors"])
      if (s.value("type", std::string()) == "lidar" && ds_export_lidar_.empty()) ds_export_lidar_ = s.value("name", std::string());
    ds_out_.clear();
    ds_export_result_ = json::object();
    DatasetRequestFrame();
  });
}

int App::DatasetFrameCount() const { return ds_info_.value("frames", 0); }

int App::DatasetFrameNumber() const {
  const json& fl = ds_info_.contains("frame_list") ? ds_info_["frame_list"] : json::array();
  if (fl.empty()) return 0;
  return fl[static_cast<size_t>(std::max(0, std::min(ds_idx_, static_cast<int>(fl.size()) - 1)))].get<int>();
}

void App::DatasetRequestFrame() {
  if (ds_root_.empty() || DatasetFrameCount() == 0) return;
  const int frame = DatasetFrameNumber();
  const std::string sensors[2] = {ds_left_, ds_right_};
  for (int i = 0; i < 2; ++i) {
    if (sensors[i].empty()) continue;
    ++ds_pending_;
    json args = {{"root", ds_root_}, {"frame", frame}, {"sensor", sensors[i]}, {"max_w", i == 0 ? 1280 : 720}, {"boxes", ds_boxes_}};
    be_.Request("dataset_frame", args, [this, i](bool ok, const json& r, const std::string& err) {
      ds_pending_ = std::max(0, ds_pending_ - 1);
      if (!ok) {
        Log(err, "error");
        ds_play_ = false;
        return;
      }
      ViewPane& p = ds_panes_[i];
      if (appui::Base64(r.value("rgb", std::string()), p.px)) {
        p.w = r.value("w", 0);
        p.h = r.value("h", 0);
        if (static_cast<int>(p.px.size()) >= p.w * p.h * 3) {
          p.dirty = true;
          ++p.frames;
        }
      }
      if (i == 0 || ds_left_.empty()) {
        ds_objects_ = r.value("objects", json::array());
        ds_frame_info_ = {{"frame", r.value("frame", 0)}, {"speed_kmh", r.value("speed_kmh", 0.0)}, {"timestamp", r.value("timestamp", 0.0)}};
      }
    });
  }
}

void App::DatasetTick() {
  if (!ds_play_ || panel_ != kPanelDataset || ds_pending_ > 0) return;
  const double now = ImGui::GetTime();
  if (now - ds_last_step_ < 1.0 / std::max(1, ds_fps_)) return;
  ds_last_step_ = now;
  if (ds_idx_ + 1 >= DatasetFrameCount()) {
    ds_play_ = false;
    return;
  }
  ++ds_idx_;
  DatasetRequestFrame();
}

// --------------------------------------------------------------------------
void App::DrawDatasetViewport(float w, float h) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.055f, 0.06f, 0.07f, 1));
  ImGui::BeginChild("dsview", ImVec2(w, h), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o = ImGui::GetWindowPos();
  if (ds_root_.empty()) {
    const char* msg = "在右侧“数据浏览”里选择一个数据集";
    const ImVec2 ts = ImGui::CalcTextSize(msg);
    dl->AddText(ImGui::GetFont(), fs * 3.0f, ImVec2(o.x + w * 0.5f - fs * 1.6f, o.y + h * 0.45f - fs * 4.0f), ImGui::GetColorU32(kHudDim), ICON_FA_FOLDER_OPEN);
    dl->AddText(ImVec2(o.x + (w - ts.x) * 0.5f, o.y + h * 0.45f), ImGui::GetColorU32(kHudText), msg);
    ImGui::EndChild();
    ImGui::PopStyleColor();
    return;
  }
  const float bar_h = fs * 2.6f, gap = 3.0f;
  const float ph = h - bar_h - gap;
  const float lw = ds_right_.empty() ? w : std::floor(w * 0.6f);
  const ImVec2 pos[2] = {o, ImVec2(o.x + lw + gap, o.y)};
  const ImVec2 size[2] = {ImVec2(ds_right_.empty() ? w : lw, ph), ImVec2(w - lw - gap, ph)};
  const std::string names[2] = {ds_left_, ds_right_};
  // Sensor list for the pickers.
  std::vector<std::string> ids, labels;
  for (const json& s : ds_info_.value("sensors", json::array())) {
    const std::string t = s.value("type", std::string());
    if (!Viewable(t)) continue;
    ids.push_back(s.value("name", std::string()));
    labels.push_back(Fmt("%s（%s）", s.value("name", std::string()).c_str(), TypeName(t)));
  }
  for (int i = 0; i < 2; ++i) {
    if (names[i].empty()) continue;
    const ImVec2 a = pos[i], e(a.x + size[i].x, a.y + size[i].y);
    dl->AddRectFilled(a, e, ImGui::GetColorU32(ImVec4(0.03f, 0.035f, 0.04f, 1)));
    const ViewPane& pane = ds_panes_[i];
    if (pane.tex && pane.w > 0 && pane.frames > 0) {
      const float s = std::min(size[i].x / pane.w, size[i].y / pane.h);
      const ImVec2 sz(pane.w * s, pane.h * s);
      const ImVec2 ia(a.x + (size[i].x - sz.x) * 0.5f, a.y + (size[i].y - sz.y) * 0.5f);
      dl->AddImage(static_cast<ImTextureID>(static_cast<intptr_t>(pane.tex)), ia, ImVec2(ia.x + sz.x, ia.y + sz.y));
    }
    dl->AddRect(a, e, ImGui::GetColorU32(ImVec4(1, 1, 1, 0.12f)));
    // Sensor picker on the label.
    std::string label = names[i];
    for (size_t k = 0; k < ids.size(); ++k)
      if (ids[k] == names[i]) label = labels[k];
    const std::string chip = label + "  " ICON_FA_CARET_DOWN;
    const ImVec2 cs = ImGui::CalcTextSize(chip.c_str());
    const ImVec2 ca(a.x + fs * 0.4f, a.y + fs * 0.4f), cb(ca.x + cs.x + fs, ca.y + fs * 1.6f);
    ImGui::SetCursorScreenPos(ca);
    ImGui::PushID(i);
    if (ImGui::InvisibleButton("##dssrc", ImVec2(cb.x - ca.x, cb.y - ca.y))) ImGui::OpenPopup("dspick");
    ui::RecordTarget(Fmt("ds:pane%d", i));
    const bool hov = ImGui::IsItemHovered();
    dl->AddRectFilled(ca, cb, ImGui::GetColorU32(hov ? ImVec4(0.2f, 0.22f, 0.25f, 0.9f) : kHudBg), 3.0f);
    dl->AddRect(ca, cb, ImGui::GetColorU32(kHudBorder), 3.0f);
    dl->AddText(ImVec2(ca.x + fs * 0.5f, ca.y + fs * 0.3f), ImGui::GetColorU32(kHudText), chip.c_str());
    if (ImGui::BeginPopup("dspick")) {
      for (size_t k = 0; k < ids.size(); ++k)
        if (ImGui::MenuItem(labels[k].c_str(), nullptr, ids[k] == names[i])) {
          (i == 0 ? ds_left_ : ds_right_) = ids[k];
          DatasetRequestFrame();
        }
      ImGui::EndPopup();
    }
    ImGui::PopID();
  }

  // Transport bar
  const ImVec2 ba(o.x, o.y + h - bar_h), be(o.x + w, o.y + h);
  dl->AddRectFilled(ba, be, ImGui::GetColorU32(p.chrome));
  dl->AddLine(ba, ImVec2(be.x, ba.y), ImGui::GetColorU32(p.card_border));
  const int n = DatasetFrameCount();
  ImGui::SetCursorScreenPos(ImVec2(ba.x + fs * 0.6f, ba.y + (bar_h - ImGui::GetFrameHeight()) * 0.5f));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 0));
  auto go = [&](int idx) {
    idx = std::max(0, std::min(n - 1, idx));
    if (idx != ds_idx_) { ds_idx_ = idx; DatasetRequestFrame(); }
  };
  if (ImGui::Button(ICON_FA_BACKWARD_FAST)) go(0);
  ui::RecordTarget("ds:first");
  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_BACKWARD_STEP)) go(ds_idx_ - 1);
  ui::RecordTarget("ds:prev");
  ImGui::SameLine();
  if (ImGui::Button(ds_play_ ? ICON_FA_PAUSE : ICON_FA_PLAY, ImVec2(fs * 2.2f, 0))) {
    if (!ds_play_ && ds_idx_ + 1 >= n) go(0);
    ds_play_ = !ds_play_;
  }
  ui::RecordTarget("ds:play");
  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_FORWARD_STEP)) go(ds_idx_ + 1);
  ui::RecordTarget("ds:next");
  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_FORWARD_FAST)) go(n - 1);
  ui::RecordTarget("ds:last");
  ImGui::SameLine(0, fs);
  const float info_w = fs * 20.0f;
  ImGui::SetNextItemWidth(std::max(fs * 6, w - ImGui::GetCursorScreenPos().x + o.x - info_w - fs));
  int idx = ds_idx_;
  if (ImGui::SliderInt("##dsframe", &idx, 0, std::max(0, n - 1), "")) go(idx);
  ImGui::SameLine(0, fs);
  ImGui::AlignTextToFramePadding();  // regular font: the monospace one has no Chinese
  ImGui::TextColored(p.text_dim, "%d/%d  帧 %d  t %.2f s  %.1f km/h", ds_idx_ + 1, n, ds_frame_info_.value("frame", 0),
                     ds_frame_info_.value("timestamp", 0.0), ds_frame_info_.value("speed_kmh", 0.0));
  ImGui::PopStyleVar();
  if (ImGui::IsWindowHovered() && !ImGui::GetIO().WantTextInput) {
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) go(ds_idx_ + 1);
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) go(ds_idx_ - 1);
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) ds_play_ = !ds_play_;
  }
  ImGui::EndChild();
  ImGui::PopStyleColor();
}

// --------------------------------------------------------------------------
void App::DrawPanelDataset() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  if (ds_sessions_.is_null() && be_.Connected()) {
    ds_sessions_ = json::array();
    DatasetRefresh();
  }

  ui::BeginCard(ICON_FA_FOLDER_OPEN, "数据集");
  const std::string dir = cfg_.contains("collect") ? cfg_["collect"].value("out_dir", std::string("datasets")) : "datasets";
  ImGui::TextColored(p.text_dim, "目录：%s（在“数据采集”页修改）", dir.c_str());
  ImGui::SameLine();
  if (ui::IconButton(ICON_FA_ROTATE, "刷新", "dsrefresh")) DatasetRefresh();
  ui::RecordTarget("ds:refresh");
  std::string to_delete;
  if (ds_sessions_.empty()) {
    ImGui::TextColored(p.text_dim, "这个目录里还没有数据集。在“数据采集”页打开采集后点“运行”。");
  } else if (ImGui::BeginTable("dssessions", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 3.0f);
    ImGui::TableSetupColumn("帧");
    ImGui::TableSetupColumn("大小");
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, fs * 1.6f);
    ImGui::TableHeadersRow();
    for (const json& s : ds_sessions_) {
      const std::string root = s.value("root", std::string());
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      if (ImGui::Selectable(Fmt("%s##%s", s.value("name", std::string()).c_str(), root.c_str()).c_str(), root == ds_root_,
                            ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
        DatasetOpen(root);
      ui::RecordTarget("ds:session:" + s.value("name", std::string()));
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s\n地图 %s · %s", root.c_str(), s.value("map", std::string()).c_str(), s.value("started", std::string()).c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%d", s.value("frames", 0));
      ImGui::TableSetColumnIndex(2);
      const double mb = s.value("size_mb", 0.0);
      ImGui::TextColored(p.text_dim, mb >= 1000 ? "%.1f GB" : "%.0f MB", mb >= 1000 ? mb / 1000 : mb);
      ImGui::TableSetColumnIndex(3);
      if (ui::IconButton(ICON_FA_TRASH, "删除这个数据集", Fmt("del%s", root.c_str()).c_str())) to_delete = root;
    }
    ImGui::EndTable();
  }
  ui::EndCard();
  if (!to_delete.empty()) {
    ds_delete_ = to_delete;
    ImGui::OpenPopup("删除数据集");
  }
  if (ImGui::BeginPopupModal("删除数据集", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("确定永久删除这个数据集？此操作不能撤销。");
    ImGui::TextColored(p.text_dim, "%s", ds_delete_.c_str());
    ImGui::Separator();
    if (ui::Button(ICON_FA_TRASH, "删除", ui::Kind::Danger)) {
      const std::string root = ds_delete_;
      Call("dataset_delete", {{"root", root}}, [this, root](const json&) {
        if (ds_root_ == root) { ds_root_.clear(); ds_info_ = json::object(); }
        DatasetRefresh();
      });
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ui::Button("", "取消")) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }

  if (ds_root_.empty()) return;

  ui::BeginCard(ICON_FA_CIRCLE_INFO, "当前数据集");
  ui::Row("地图");
  ImGui::TextUnformatted(ds_info_.value("map", std::string()).c_str());
  ui::Row("帧数");
  ImGui::Text("%d（帧号 %d – %d）", DatasetFrameCount(), ds_info_.value("first", 0), ds_info_.value("last", 0));
  ui::Row("传感器");
  std::string sl;
  for (const json& s : ds_info_.value("sensors", json::array()))
    sl += Fmt("%s（%s）  ", s.value("name", std::string()).c_str(), TypeName(s.value("type", std::string())));
  ImGui::TextWrapped("%s", sl.c_str());
  ui::Row("显示真值框");
  if (ImGui::Checkbox("##dsboxes", &ds_boxes_)) DatasetRequestFrame();
  ui::Row("播放速度", nullptr, fs * 10);
  ImGui::SliderInt("##dsfps", &ds_fps_, 1, 20, "%d 帧/秒");
  ImGui::TextColored(p.text_dim, ICON_FA_CIRCLE_INFO "  画面在中间的视口里；← → 逐帧，空格播放 / 暂停，点视图标签换传感器。");
  ui::EndCard();

  ui::BeginCard(ICON_FA_TAGS, Fmt("当前帧目标（%d 个）", static_cast<int>(ds_objects_.size())).c_str(), "dsobjs");
  if (ds_objects_.empty()) {
    ImGui::TextColored(p.text_dim, "这一帧没有真值目标（或采集时没打开真值标注）");
  } else if (ImGui::BeginTable("dsobjtab", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_ScrollY |
                                                   ImGuiTableFlags_SizingStretchProp, ImVec2(0, fs * 12))) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("ID");
    ImGui::TableSetupColumn("类别", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("距离 m");
    ImGui::TableSetupColumn("框内激光点");
    ImGui::TableHeadersRow();
    for (const json& ob : ds_objects_) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0); ImGui::Text("%d", ob.value("id", 0));
      ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(ob.value("class", std::string()).c_str());
      ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", ob.value("distance", 0.0));
      ImGui::TableSetColumnIndex(3);
      if (ob["lidar_pts"].is_null()) ImGui::TextDisabled("-"); else ImGui::Text("%d", ob["lidar_pts"].get<int>());
    }
    ImGui::EndTable();
  }
  ui::EndCard();

  ui::BeginCard(ICON_FA_FILE_EXPORT, "导出");
  ui::Row("格式", "KITTI：一个前视相机 + 一个激光雷达，3D 目标检测常用；nuScenes：全部相机、激光雷达、毫米波雷达，带时序和实例跟踪");
  ImGui::BeginDisabled(ds_exporting_);
  if (ImGui::RadioButton("KITTI", ds_fmt_ == 0)) ds_fmt_ = 0;
  ImGui::SameLine();
  if (ImGui::RadioButton("nuScenes", ds_fmt_ == 1)) ds_fmt_ = 1;
  std::vector<std::string> cams, lidars;
  for (const json& s : ds_info_.value("sensors", json::array())) {
    if (s.value("type", std::string()) == "rgb") cams.push_back(s.value("name", std::string()));
    if (s.value("type", std::string()) == "lidar") lidars.push_back(s.value("name", std::string()));
  }
  if (ds_fmt_ == 0) {
    ui::Row("相机 → image_2", nullptr, fs * 14);
    ComboStr("##dscam", ds_export_cam_, cams);
    ui::Row("激光雷达 → velodyne", nullptr, fs * 14);
    ComboStr("##dslid", ds_export_lidar_, lidars);
    ui::Row("最少激光点", "框内激光点少于这个数的目标不写进 label_2（看不见的目标）", fs * 8);
    ImGui::InputInt("##dsminpts", &ds_min_pts_);
    ds_min_pts_ = std::max(0, ds_min_pts_);
    if (cams.empty() || lidars.empty())
      ImGui::TextColored(p.warning, ICON_FA_TRIANGLE_EXCLAMATION "  KITTI 需要至少一个 RGB 相机和一个激光雷达");
  }
  const std::string def_out = ds_root_ + (ds_fmt_ == 0 ? "_kitti" : "_nuscenes");
  ui::Row("输出目录", "必须是空目录或不存在；默认在数据集旁边");
  std::string out = ds_out_.empty() ? def_out : ds_out_;
  if (InputStr("##dsout", out)) ds_out_ = out == def_out ? std::string() : out;
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  const bool can = ds_fmt_ == 1 || (!cams.empty() && !lidars.empty());
  ImGui::BeginDisabled(!can);
  if (ui::Button(ICON_FA_FILE_EXPORT, "导出", ui::Kind::Primary)) {
    json args = {{"root", ds_root_}, {"format", ds_fmt_ == 0 ? "kitti" : "nuscenes"}, {"out", out}};
    if (ds_fmt_ == 0) {
      args["camera"] = ds_export_cam_;
      args["lidar"] = ds_export_lidar_;
      args["min_lidar_pts"] = ds_min_pts_;
    }
    ds_export_result_ = json::object();
    ds_export_done_ = 0;
    Call("dataset_export", args, [this](const json& r) {
      ds_exporting_ = true;
      ds_export_total_ = r.value("frames", 0);
      Log(Fmt("开始导出到 %s（预计 %.0f MB）", r.value("out", std::string()).c_str(), r.value("estimate_mb", 0.0)));
    });
  }
  ui::RecordTarget("ds:export");
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  if (ds_exporting_) {
    ImGui::ProgressBar(ds_export_total_ ? static_cast<float>(ds_export_done_) / ds_export_total_ : 0.0f, ImVec2(-1, 0),
                       Fmt("%d / %d 帧", ds_export_done_, ds_export_total_).c_str());
  }
  if (!ds_export_result_.empty()) {
    if (ds_export_result_.value("ok", false)) {
      const json& r = ds_export_result_["result"];
      ImGui::TextColored(p.success, ICON_FA_CIRCLE_CHECK "  已导出 %d 帧、%d 个目标，%.0f MB", r.value("frames", 0), r.value("objects", 0),
                         r.value("size_mb", 0.0));
      ImGui::TextWrapped("%s", r.value("out", std::string()).c_str());
      if (r.value("format", std::string()) == "nuscenes")
        ImGui::TextColored(p.text_dim, "用 NuScenes(version='%s', dataroot='<上面的目录>') 读取", r.value("version", std::string()).c_str());
    } else {
      ImGui::TextColored(p.danger, ICON_FA_CIRCLE_XMARK "  导出失败：%s", ds_export_result_.value("error", std::string()).c_str());
    }
  }
  ui::EndCard();
}
