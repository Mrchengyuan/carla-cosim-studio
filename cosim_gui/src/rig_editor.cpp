// Sensor rig editor: presets, drag-and-drop mounting on top / side views,
// per-sensor properties and a data-volume estimate.
#include <algorithm>
#include <cmath>

#include "app.h"
#include "imgui.h"
#include "ui_kit.h"

using appui::ComboStr;
using appui::Fmt;
using appui::InputStr;

namespace {

constexpr float kPi = 3.14159265f;

// Same numbers as rig.py (measured from a 3-frame capture in Town10HD).
// dt: the simulation step; collected lidar sweeps once per frame (points_per_second x dt).
double BytesPerFrame(const json& s, const std::string& image_format, double dt) {
  const std::string k = s.value("type", std::string());
  const json& a = s.contains("attributes") ? s["attributes"] : json::object();
  if (k == "rgb" || k == "depth" || k == "semantic" || k == "instance") {
    const double px3 = a.value("image_size_x", 800.0) * a.value("image_size_y", 600.0) * 3.0;
    if (k == "rgb") return px3 * (image_format == "jpg" ? 0.108 : 0.45);
    if (k == "depth") return px3 * 4.0 / 3.0;  // float32 metres (.npy)
    return px3 * (k == "semantic" ? 0.016 : 0.022);
  }
  if (k == "lidar") return a.value("points_per_second", 56000.0) * dt * 0.5 * 16;
  if (k == "radar") return a.value("points_per_second", 1500.0) / 10.0 * 32;
  return 200;
}

ImVec4 TypeColor(const std::string& t) {
  if (t == "rgb") return ImVec4(0.26f, 0.58f, 0.98f, 1);
  if (t == "depth") return ImVec4(0.62f, 0.45f, 0.95f, 1);
  if (t == "semantic" || t == "instance") return ImVec4(0.95f, 0.55f, 0.25f, 1);
  if (t == "lidar") return ImVec4(0.20f, 0.80f, 0.55f, 1);
  if (t == "radar") return ImVec4(0.95f, 0.35f, 0.45f, 1);
  return ImVec4(0.75f, 0.75f, 0.80f, 1);
}

const char* TypeIcon(const std::string& t) {
  if (t == "lidar") return ICON_FA_CIRCLE_NODES;
  if (t == "radar") return ICON_FA_WIFI;
  if (t == "imu") return ICON_FA_GAUGE_HIGH;
  if (t == "gnss") return ICON_FA_LOCATION_DOT;
  return ICON_FA_CAMERA;
}

const char* TypeName(const std::string& t) {
  if (t == "rgb") return "RGB 相机";
  if (t == "depth") return "深度相机";
  if (t == "semantic") return "语义分割";
  if (t == "instance") return "实例分割";
  if (t == "lidar") return "激光雷达";
  if (t == "radar") return "毫米波雷达";
  if (t == "imu") return "IMU";
  if (t == "gnss") return "GNSS";
  return t.c_str();
}

// CarSim reference point (origin of the rig's CarSim vehicle frame) in the
// CARLA vehicle frame the drawings use: x forward, y RIGHT, z up, m. Same as
// rig.preset_reference in the backend (front axle: z = 0, on the ground).
struct Ref { float x, y, z; };

Ref RefPoint(const json& cfg, const json* spec) {
  if (cfg.contains("sync") && cfg["sync"].contains("reference_point")) {
    const json& r = cfg["sync"]["reference_point"];
    if (r.is_array() && r.size() >= 3)
      return {static_cast<float>(appui::NumAt(r, 0)), static_cast<float>(appui::NumAt(r, 1)), static_cast<float>(appui::NumAt(r, 2))};
  }
  return {spec ? spec->value("front_axle_x_m", 1.4f) : 1.4f, 0.0f, spec ? spec->value("front_axle_z_m", 0.0f) : 0.0f};
}

json DefaultSensor(const std::string& type, const std::string& name, const json* spec, Ref ref) {
  const double L = spec ? spec->value("length_m", 4.8) : 4.8;
  const double H = spec ? spec->value("height_m", 1.5) : 1.5;
  json a = json::object();
  double x = L * 0.15, z = H * 0.92;
  if (type == "rgb" || type == "depth" || type == "semantic" || type == "instance") {
    a = {{"image_size_x", 1280}, {"image_size_y", 720}, {"fov", 90.0}};
    if (type == "depth") a["max_distance"] = 100.0;
  } else if (type == "lidar") {
    a = {{"channels", 32}, {"range", 100.0}, {"points_per_second", 600000}, {"rotation_frequency", 10.0},
         {"upper_fov", 10.0}, {"lower_fov", -30.0}};
    x = 0.0;
    z = H + 0.3;
  } else if (type == "radar") {
    a = {{"horizontal_fov", 30.0}, {"vertical_fov", 10.0}, {"range", 100.0}, {"points_per_second", 1500}};
    x = L / 2.0;
    z = 0.5;
  } else {
    x = 0.0;
    z = H * 0.5;
  }
  // Positions above are relative to the car centre: to CarSim's vehicle frame.
  return {{"name", name}, {"type", type}, {"x", std::round((x - ref.x) * 100.0) / 100.0}, {"y", ref.y},
          {"z", std::round((z - ref.z) * 100.0) / 100.0}, {"roll", 0.0}, {"pitch", 0.0},
          {"yaw", 0.0}, {"attributes", a}, {"enabled", true}};
}

struct Dims { float L, W, H, wb, track, r, fx; };

Dims VehicleDims(const json* s) {
  Dims d{4.8f, 2.0f, 1.5f, 2.9f, 1.6f, 0.35f, 1.4f};  // fx as rig.FRONT_AXLE_X
  if (!s) return d;
  d.L = s->value("length_m", d.L);
  d.W = s->value("width_m", d.W);
  d.H = s->value("height_m", d.H);
  d.wb = s->value("wheelbase_m", d.wb);
  d.track = s->value("track_m", d.track);
  d.fx = s->value("front_axle_x_m", d.fx);
  const json wr = s->value("wheel_radius_m", json::array());
  if (wr.is_array() && !wr.empty()) d.r = static_cast<float>(appui::NumAt(wr, 0));  // null (NaN) safe
  return d;
}

}  // namespace

json& App::RigSensors() {
  if (!cfg_.contains("rig")) cfg_["rig"] = {{"preset", "front_camera"}, {"sensors", json::array()}};
  if (!cfg_["rig"].contains("sensors") || !cfg_["rig"]["sensors"].is_array()) cfg_["rig"]["sensors"] = json::array();
  return cfg_["rig"]["sensors"];
}

void App::LoadRigPreset(const std::string& preset) {
  Call("rig_build", {{"preset", preset}, {"blueprint", cfg_["carla"].value("vehicle", std::string())},
                     {"reference_point", cfg_["sync"].value("reference_point", json("front_axle"))}},
       [this, preset](const json& r) {
         RigSensors() = r;
         cfg_["rig"]["preset"] = preset;
         rig_sel_ = r.empty() ? -1 : 0;
         RefreshVehicles();  // rig_build may have measured the vehicle
         Log(Fmt("已加载传感器套件：%d 个传感器", static_cast<int>(r.size())));
       }, "正在按车型尺寸生成套件 ...");
}

void App::AddRigSensor(const std::string& type) {
  json& s = RigSensors();
  int n = 1;
  std::string name;
  do {
    name = Fmt("%s_%d", type.c_str(), n++);
  } while (std::any_of(s.begin(), s.end(), [&](const json& x) { return x.value("name", std::string()) == name; }));
  s.push_back(DefaultSensor(type, name, SelectedVehicleSpec(), RefPoint(cfg_, SelectedVehicleSpec())));
  rig_sel_ = static_cast<int>(s.size()) - 1;
}

// --------------------------------------------------------------------------
void App::DrawPanelRig() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  json& sensors = RigSensors();

  ui::BeginCard(ICON_FA_SATELLITE_DISH, "传感器套件");
  ui::Row("预设方案", "安装位置会按当前车型的长宽高自动计算");
  std::vector<std::string> ids, names;
  for (const json& pr : rig_presets_) {
    ids.push_back(pr.value("id", std::string()));
    names.push_back(pr.value("name", std::string()));
  }
  {
    const float bw = ImGui::CalcTextSize(ICON_FA_WAND_MAGIC_SPARKLES "  加载预设").x + ImGui::GetStyle().FramePadding.x * 2;
    ImGui::SetNextItemWidth(std::min(fs * 26, ImGui::GetContentRegionAvail().x - bw - ImGui::GetStyle().ItemSpacing.x));
  }
  ComboStr("##preset", rig_preset_choice_, ids, &names);
  ImGui::SameLine();
  ImGui::BeginDisabled(!busy_.empty() || Running());
  if (ui::Button(ICON_FA_WAND_MAGIC_SPARKLES, "加载预设", ui::Kind::Primary)) LoadRigPreset(rig_preset_choice_);
  ImGui::EndDisabled();
  ui::Row("添加传感器");
  struct A { const char* type; const char* label; };
  static const A kAdd[] = {{"rgb", "相机"}, {"depth", "深度"}, {"semantic", "语义"}, {"instance", "实例"},
                           {"lidar", "激光雷达"}, {"radar", "毫米波"}, {"imu", "IMU"}, {"gnss", "GNSS"}};
  ImGui::BeginDisabled(Running());
  ImGui::BeginGroup();
  // Wrap onto the next line when the properties panel is narrow.
  const float right_edge = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
  for (int i = 0; i < 8; ++i) {
    const std::string label = Fmt("%s %s", TypeIcon(kAdd[i].type), kAdd[i].label);
    const float bw = ImGui::CalcTextSize(label.c_str()).x + ImGui::GetStyle().FramePadding.x * 2;
    if (i && ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + bw < right_edge) ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, TypeColor(kAdd[i].type));
    if (ImGui::Button(label.c_str())) AddRigSensor(kAdd[i].type);
    ImGui::PopStyleColor();
  }
  ImGui::EndGroup();
  ImGui::EndDisabled();
  const json* spec = SelectedVehicleSpec();
  if (!spec) {
    ImGui::TextColored(p.warning, ICON_FA_CIRCLE_INFO "  当前车型还没测量尺寸，按 4.8 × 2.0 × 1.5 m 的轿车画图；"
                                  "在“车辆与视角”页点“测量”后会更准确");
  }
  ui::EndCard();

  // ---- views ----------------------------------------------------------------
  ui::BeginCard(ICON_FA_CAR, "安装位置（拖动传感器调整）");
  const float avail = ImGui::GetContentRegionAvail().x;
  // Side by side when there is room, stacked in a narrow properties panel.
  const bool stacked = avail < fs * 40;
  const float top_w = stacked ? avail : avail * 0.5f - ImGui::GetStyle().ItemSpacing.x * 0.5f;
  ImGui::BeginGroup();
  ImGui::TextColored(p.text_dim, "俯视图（车头朝上，x 向前，y 向左）");
  DrawRigTopView(top_w, fs * (stacked ? 17 : 20));
  ImGui::EndGroup();
  if (!stacked) ImGui::SameLine();
  ImGui::BeginGroup();
  ImGui::TextColored(p.text_dim, "侧视图（车头朝右，z 向上）");
  DrawRigSideView(top_w, fs * 12);
  ImGui::TextColored(p.text_dim, "网格间距 1 m · 扇形 = 视场角 · 圆环 = 激光雷达 · 坐标按 CarSim 车身坐标系，原点 = 参考点（蓝色十字）");
  ImGui::EndGroup();
  ui::EndCard();

  // ---- list + properties ----------------------------------------------------
  ui::BeginCard(ICON_FA_LIST, Fmt("传感器列表（%d 个）", static_cast<int>(sensors.size())).c_str(), "rig_list");
  const std::string imf = cfg_.contains("collect") ? cfg_["collect"].value("image_format", std::string("jpg")) : "jpg";
  if (ImGui::BeginTable("rigtab", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, fs * 1.5f);
    ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("类型");
    ImGui::TableSetupColumn("位置 x, y, z (m)", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("朝向 °");
    ImGui::TableSetupColumn("参数", ImGuiTableColumnFlags_WidthStretch, 2.0f);
    ImGui::TableSetupColumn("MB/帧");
    ImGui::TableHeadersRow();
    for (int i = 0; i < static_cast<int>(sensors.size()); ++i) {
      json& s = sensors[static_cast<size_t>(i)];
      const std::string t = s.value("type", std::string());
      const json& a = s["attributes"];
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      bool en = s.value("enabled", true);
      if (ImGui::Checkbox(Fmt("##en%d", i).c_str(), &en)) s["enabled"] = en;
      ImGui::TableSetColumnIndex(1);
      ImGui::PushStyleColor(ImGuiCol_Text, TypeColor(t));
      ImGui::TextUnformatted(TypeIcon(t));
      ImGui::PopStyleColor();
      ImGui::SameLine();
      if (ImGui::Selectable(Fmt("%s##s%d", s.value("name", std::string()).c_str(), i).c_str(), rig_sel_ == i,
                            ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
        rig_sel_ = i;
      ImGui::TableSetColumnIndex(2); ImGui::TextColored(p.text_dim, "%s", TypeName(t));
      ImGui::TableSetColumnIndex(3); ImGui::Text("%.2f, %.2f, %.2f", s.value("x", 0.0), s.value("y", 0.0), s.value("z", 0.0));
      ImGui::TableSetColumnIndex(4); ImGui::Text("%.0f", s.value("yaw", 0.0));
      ImGui::TableSetColumnIndex(5);
      if (t == "rgb" || t == "depth" || t == "semantic" || t == "instance")
        ImGui::Text("%d×%d  FOV %.0f°", a.value("image_size_x", 0), a.value("image_size_y", 0), a.value("fov", 0.0));
      else if (t == "lidar")
        ImGui::Text("%d 线  %.0f m  %.0f 万点/秒", a.value("channels", 0), a.value("range", 0.0), a.value("points_per_second", 0.0) / 1e4);
      else if (t == "radar")
        ImGui::Text("%.0f°×%.0f°  %.0f m", a.value("horizontal_fov", 0.0), a.value("vertical_fov", 0.0), a.value("range", 0.0));
      else
        ImGui::TextColored(p.text_dim, "-");
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%.2f", BytesPerFrame(s, imf, cfg_["sync"].value("frame_dt", 0.02)) / 1e6);
    }
    ImGui::EndTable();
  }
  if (sensors.empty()) ImGui::TextColored(p.text_dim, "还没有传感器：选择一个预设，或者点上面的“添加”按钮");
  ui::EndCard();

  if (rig_sel_ >= 0 && rig_sel_ < static_cast<int>(sensors.size())) DrawRigProperties();
  DrawRigEstimate(true);
}

// --------------------------------------------------------------------------
void App::DrawRigTopView(float w, float h) {
  const ui::Palette& p = ui::Colors();
  json& sensors = RigSensors();
  const Dims d = VehicleDims(SelectedVehicleSpec());
  const Ref ref = RefPoint(cfg_, SelectedVehicleSpec());
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o = ImGui::GetCursorScreenPos();
  // The sensor handles are drawn on top of this canvas: let them take the mouse.
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("topview", ImVec2(w, h));
  const bool canvas_hovered = ImGui::IsItemHovered();
  dl->AddRectFilled(o, ImVec2(o.x + w, o.y + h), ImGui::GetColorU32(p.plot_bg), 2.0f);
  dl->AddRect(o, ImVec2(o.x + w, o.y + h), ImGui::GetColorU32(p.card_border), 2.0f);
  const float scale = std::min(w / (d.W + 5.0f), h / (d.L + 3.0f));  // px per meter
  const ImVec2 c(o.x + w * 0.5f, o.y + h * 0.5f);
  auto to_px = [&](float x, float y) { return ImVec2(c.x + y * scale, c.y - x * scale); };
  dl->PushClipRect(o, ImVec2(o.x + w, o.y + h), true);
  // 1 m grid
  const ImU32 grid = ImGui::GetColorU32(ImVec4(p.text_dim.x, p.text_dim.y, p.text_dim.z, 0.12f));
  for (int k = -20; k <= 20; ++k) {
    dl->AddLine(to_px(-20.f, static_cast<float>(k)), to_px(20.f, static_cast<float>(k)), grid);
    dl->AddLine(to_px(static_cast<float>(k), -20.f), to_px(static_cast<float>(k), 20.f), grid);
  }
  // Vehicle body, windshield and wheels
  const ImU32 body = ImGui::GetColorU32(ImVec4(p.text.x, p.text.y, p.text.z, 0.10f));
  const ImU32 edge = ImGui::GetColorU32(ImVec4(p.text.x, p.text.y, p.text.z, 0.55f));
  dl->AddRectFilled(to_px(d.L / 2, -d.W / 2), to_px(-d.L / 2, d.W / 2), body, scale * 0.35f);
  dl->AddRect(to_px(d.L / 2, -d.W / 2), to_px(-d.L / 2, d.W / 2), edge, scale * 0.35f, 0, 1.5f);
  dl->AddLine(to_px(d.L * 0.18f, -d.W * 0.42f), to_px(d.L * 0.18f, d.W * 0.42f), edge, 1.2f);
  const float rx = d.fx - d.wb;
  for (float wx : {d.fx, rx})
    for (float wy : {-d.track / 2, d.track / 2})
      dl->AddRectFilled(to_px(wx + d.r, wy - 0.12f), to_px(wx - d.r, wy + 0.12f), edge, 2.0f);
  dl->AddTriangleFilled(to_px(d.L / 2 + 0.35f, 0), to_px(d.L / 2 + 0.05f, -0.22f), to_px(d.L / 2 + 0.05f, 0.22f),
                        ImGui::GetColorU32(p.accent));
  // CarSim reference point with its axes: x forward (up), y LEFT.
  {
    const ImU32 ac = ImGui::GetColorU32(ImVec4(0.30f, 0.62f, 1.0f, 1));
    const ImVec2 r0 = to_px(ref.x, ref.y), rx = to_px(ref.x + 1.2f, ref.y), ry = to_px(ref.x, ref.y - 1.2f);
    dl->AddLine(r0, rx, ac, 2.0f);
    dl->AddLine(r0, ry, ac, 2.0f);
    dl->AddText(ImVec2(rx.x + 3, rx.y - 8), ac, "x");
    dl->AddText(ImVec2(ry.x - 12, ry.y - 16), ac, "y");
    dl->AddCircleFilled(r0, 3.5f, ac);
  }

  // Sensors: fov wedge / lidar ring, then draggable handles on top.
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < static_cast<int>(sensors.size()); ++i) {
      json& s = sensors[static_cast<size_t>(i)];
      if (!s.value("enabled", true)) continue;
      const std::string t = s.value("type", std::string());
      const ImVec4 col = TypeColor(t);
      const bool sel = i == rig_sel_;
      // CarSim mount (y left, yaw + = left) -> the drawing's CARLA frame.
      const float cx = s.value("x", 0.0f), cy = s.value("y", 0.0f);
      const float sx = cx + ref.x, sy = ref.y - cy;
      const ImVec2 pp = to_px(sx, sy);
      if (pass == 0) {
        const json& a = s["attributes"];
        const float yaw = -s.value("yaw", 0.0f) * kPi / 180.0f;
        float fov = 0, len = 0;
        if (t == "rgb" || t == "depth" || t == "semantic" || t == "instance") { fov = a.value("fov", 90.0f); len = 3.2f; }
        if (t == "radar") { fov = a.value("horizontal_fov", 30.0f); len = 3.6f; }
        ImVec4 fill = col;
        fill.w = sel ? 0.28f : 0.12f;
        if (fov > 0) {
          const float half = fov * 0.5f * kPi / 180.0f;
          const int seg = 16;
          dl->PathLineTo(pp);
          for (int k = 0; k <= seg; ++k) {
            const float ang = yaw - half + 2 * half * k / seg;
            dl->PathLineTo(to_px(sx + len * std::cos(ang), sy + len * std::sin(ang)));
          }
          dl->PathFillConvex(ImGui::GetColorU32(fill));
        } else if (t == "lidar") {
          dl->AddCircle(pp, 2.2f * scale, ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, sel ? 0.8f : 0.45f)), 48, 1.5f);
          dl->AddCircle(pp, 1.3f * scale, ImGui::GetColorU32(ImVec4(col.x, col.y, col.z, sel ? 0.5f : 0.25f)), 48, 1.0f);
        }
      } else {
        const float r = sel ? 7.0f : 5.0f;
        dl->AddCircleFilled(pp, r, ImGui::GetColorU32(col));
        dl->AddCircle(pp, r, ImGui::GetColorU32(ImVec4(1, 1, 1, sel ? 1.0f : 0.6f)), 0, sel ? 2.0f : 1.0f);
        ImGui::SetCursorScreenPos(ImVec2(pp.x - 9, pp.y - 9));
        ImGui::InvisibleButton(Fmt("h%d", i).c_str(), ImVec2(18, 18));
        if (ImGui::IsItemActivated()) rig_sel_ = i;
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0, 0.0f) && !Running()) {
          const ImVec2 dd = ImGui::GetIO().MouseDelta;
          s["x"] = std::round((cx - dd.y / scale) * 100.0f) / 100.0f;
          s["y"] = std::round((cy - dd.x / scale) * 100.0f) / 100.0f;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s  (%.2f, %.2f, %.2f)", s.value("name", std::string()).c_str(), cx, cy, s.value("z", 0.0f));
        if (sel) dl->AddText(ImVec2(pp.x + 9, pp.y - 8), ImGui::GetColorU32(p.text), s.value("name", std::string()).c_str());
      }
    }
  }
  dl->PopClipRect();
  (void)canvas_hovered;
  ImGui::SetCursorScreenPos(ImVec2(o.x, o.y + h + ImGui::GetStyle().ItemSpacing.y));
  ImGui::Dummy(ImVec2(w, 0));
}

void App::DrawRigSideView(float w, float h) {
  const ui::Palette& p = ui::Colors();
  json& sensors = RigSensors();
  const Dims d = VehicleDims(SelectedVehicleSpec());
  const Ref ref = RefPoint(cfg_, SelectedVehicleSpec());
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 o = ImGui::GetCursorScreenPos();
  // The sensor handles are drawn on top of this canvas: let them take the mouse.
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("sideview", ImVec2(w, h));
  dl->AddRectFilled(o, ImVec2(o.x + w, o.y + h), ImGui::GetColorU32(p.plot_bg), 2.0f);
  dl->AddRect(o, ImVec2(o.x + w, o.y + h), ImGui::GetColorU32(p.card_border), 2.0f);
  const float scale = std::min(w / (d.L + 3.0f), h / (d.H + 1.6f));
  const ImVec2 g(o.x + w * 0.5f, o.y + h - scale * 0.45f);   // ground under the car centre
  auto to_px = [&](float x, float z) { return ImVec2(g.x + x * scale, g.y - z * scale); };
  dl->PushClipRect(o, ImVec2(o.x + w, o.y + h), true);
  const ImU32 grid = ImGui::GetColorU32(ImVec4(p.text_dim.x, p.text_dim.y, p.text_dim.z, 0.12f));
  for (int k = -10; k <= 10; ++k) dl->AddLine(to_px(static_cast<float>(k), -1.f), to_px(static_cast<float>(k), 5.f), grid);
  for (int k = 0; k <= 5; ++k) dl->AddLine(to_px(-10.f, static_cast<float>(k)), to_px(10.f, static_cast<float>(k)), grid);
  dl->AddLine(to_px(-10, 0), to_px(10, 0), ImGui::GetColorU32(p.text_dim), 1.5f);
  // Body silhouette: lower box + cabin
  const ImU32 body = ImGui::GetColorU32(ImVec4(p.text.x, p.text.y, p.text.z, 0.10f));
  const ImU32 edge = ImGui::GetColorU32(ImVec4(p.text.x, p.text.y, p.text.z, 0.55f));
  const float sill = d.r * 0.9f, belt = d.H * 0.58f;
  dl->AddRectFilled(to_px(-d.L / 2, sill), to_px(d.L / 2, belt), body, scale * 0.25f);
  dl->AddRect(to_px(-d.L / 2, sill), to_px(d.L / 2, belt), edge, scale * 0.25f, 0, 1.5f);
  ImVec2 cabin[4] = {to_px(-d.L * 0.36f, belt), to_px(-d.L * 0.22f, d.H), to_px(d.L * 0.12f, d.H), to_px(d.L * 0.26f, belt)};
  dl->AddConvexPolyFilled(cabin, 4, body);
  dl->AddPolyline(cabin, 4, edge, 0, 1.5f);
  const float rx = d.fx - d.wb;
  for (float wx : {d.fx, rx}) {
    dl->AddCircleFilled(to_px(wx, d.r), d.r * scale, ImGui::GetColorU32(ImVec4(0.2f, 0.2f, 0.22f, 1)));
    dl->AddCircle(to_px(wx, d.r), d.r * scale * 0.55f, edge, 0, 1.2f);
  }
  dl->AddCircleFilled(to_px(ref.x, ref.z), 3.5f, ImGui::GetColorU32(ImVec4(0.30f, 0.62f, 1.0f, 1)));  // reference point
  for (int i = 0; i < static_cast<int>(sensors.size()); ++i) {
    json& s = sensors[static_cast<size_t>(i)];
    if (!s.value("enabled", true)) continue;
    const std::string t = s.value("type", std::string());
    const ImVec4 col = TypeColor(t);
    const bool sel = i == rig_sel_;
    const float cx = s.value("x", 0.0f), sx = cx + ref.x, cz = s.value("z", 0.0f), sz = cz + ref.z;
    const ImVec2 pp = to_px(sx, sz);
    const bool camera = t == "rgb" || t == "depth" || t == "semantic" || t == "instance";
    if (camera && std::fabs(std::fabs(s.value("yaw", 0.0f)) - 90.0f) > 20.0f) {
      // Vertical field of view projected on the side view (forward or backward facing).
      const json& a = s["attributes"];
      const float aspect = a.value("image_size_y", 720.0f) / std::max(1.0f, a.value("image_size_x", 1280.0f));
      const float hfov = a.value("fov", 90.0f) * kPi / 180.0f;
      const float vhalf = std::atan(std::tan(hfov / 2) * aspect);
      const float dir = std::fabs(s.value("yaw", 0.0f)) > 90.0f ? -1.0f : 1.0f;
      const float pitch = -s.value("pitch", 0.0f) * kPi / 180.0f;  // CarSim: + = nose down
      const float len = 2.4f;
      ImVec4 fill = col;
      fill.w = sel ? 0.28f : 0.12f;
      dl->AddTriangleFilled(pp, to_px(sx + dir * len * std::cos(pitch + vhalf), sz + len * std::sin(pitch + vhalf)),
                            to_px(sx + dir * len * std::cos(pitch - vhalf), sz + len * std::sin(pitch - vhalf)),
                            ImGui::GetColorU32(fill));
    }
    const float r = sel ? 7.0f : 5.0f;
    dl->AddCircleFilled(pp, r, ImGui::GetColorU32(col));
    dl->AddCircle(pp, r, ImGui::GetColorU32(ImVec4(1, 1, 1, sel ? 1.0f : 0.6f)), 0, sel ? 2.0f : 1.0f);
    ImGui::SetCursorScreenPos(ImVec2(pp.x - 9, pp.y - 9));
    ImGui::InvisibleButton(Fmt("sv%d", i).c_str(), ImVec2(18, 18));
    if (ImGui::IsItemActivated()) rig_sel_ = i;
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0, 0.0f) && !Running()) {
      const ImVec2 dd = ImGui::GetIO().MouseDelta;
      s["x"] = std::round((cx + dd.x / scale) * 100.0f) / 100.0f;
      s["z"] = std::round((std::max(0.0f, sz - dd.y / scale) - ref.z) * 100.0f) / 100.0f;  // not below the ground
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s  z %.2f m（离地 %.2f m）", s.value("name", std::string()).c_str(), cz, sz);
  }
  dl->PopClipRect();
  ImGui::SetCursorScreenPos(ImVec2(o.x, o.y + h + ImGui::GetStyle().ItemSpacing.y));
  ImGui::Dummy(ImVec2(w, 0));
}

// --------------------------------------------------------------------------
void App::DrawRigProperties() {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  json& sensors = RigSensors();
  json& s = sensors[static_cast<size_t>(rig_sel_)];
  const std::string t = s.value("type", std::string());
  json& a = s["attributes"];
  ui::BeginCard(TypeIcon(t), Fmt("%s 属性", s.value("name", std::string()).c_str()).c_str(), "rig_props");
  ImGui::BeginDisabled(Running());
  std::string name = s.value("name", std::string());
  ui::Row("名称", "也是输出文件夹名");
  if (InputStr("##name", name)) s["name"] = name;
  ui::Row("类型");
  ImGui::TextColored(TypeColor(t), "%s", TypeName(t));

  auto drag = [&](const char* label, const char* key, float speed, float lo, float hi, const char* fmt, const char* help) {
    float v = s.value(key, 0.0f);
    ui::Row(label, help, fs * 10);
    if (ImGui::DragFloat(Fmt("##%s", key).c_str(), &v, speed, lo, hi, fmt)) s[key] = v;
  };
  drag("x 向前 m", "x", 0.01f, -10, 10, "%.2f", "CarSim 车身坐标系：原点在 CarSim 参考点（默认前轴中心的地面）");
  drag("y 向左 m", "y", 0.01f, -5, 5, "%.2f", "左为正，右为负（和 CarSim 一样）");
  drag("z 向上 m", "z", 0.01f, -3, 6, "%.2f", "相对参考点的高度（参考点默认在前轴中心的地面，这时就是离地高度）");
  drag("航向 °", "yaw", 0.5f, -180, 180, "%.1f", "0 = 朝前，90 = 朝左，-90 = 朝右，180 = 朝后（左为正，和 CarSim 一样）");
  drag("俯仰 °", "pitch", 0.2f, -90, 90, "%.1f", "正数 = 向下看（CarSim：俯仰正 = 低头）");
  drag("侧倾 °", "roll", 0.2f, -180, 180, "%.1f", nullptr);

  auto attr_int = [&](const char* label, const char* key, int step, const char* help = nullptr) {
    int v = a.value(key, 0);
    ui::Row(label, help, fs * 8);
    if (ImGui::InputInt(Fmt("##a%s", key).c_str(), &v, step)) a[key] = std::max(1, v);
  };
  auto attr_float = [&](const char* label, const char* key, float lo, float hi, const char* fmt, const char* help = nullptr) {
    float v = a.value(key, 0.0f);
    ui::Row(label, help, fs * 12);
    if (ImGui::SliderFloat(Fmt("##a%s", key).c_str(), &v, lo, hi, fmt)) a[key] = v;
  };
  if (t == "rgb" || t == "depth" || t == "semantic" || t == "instance") {
    static const int kRes[][2] = {{640, 360}, {1280, 720}, {1600, 900}, {1920, 1080}, {1242, 375}, {2560, 1440}};
    ui::Row("分辨率预设");
    for (int i = 0; i < 6; ++i) {
      if (i) ImGui::SameLine();
      if (ImGui::SmallButton(Fmt("%d×%d", kRes[i][0], kRes[i][1]).c_str())) {
        a["image_size_x"] = kRes[i][0];
        a["image_size_y"] = kRes[i][1];
      }
    }
    attr_int("宽 px", "image_size_x", 16);
    attr_int("高 px", "image_size_y", 16);
    attr_float("水平视场角 °", "fov", 10, 170, "%.0f", "常见：长焦 30，标准 60-90，广角 120，鱼眼 170");
    if (t == "depth")
      attr_float("最大距离 m", "max_distance", 10, 1000, "%.0f", "超过这个距离的像素记为这个距离（深度以米为单位保存）");
  } else if (t == "lidar") {
    ui::Row("线数");
    for (int ch : {16, 32, 64, 128}) {
      ImGui::SameLine(0, 0);
      if (ImGui::RadioButton(Fmt("%d  ##ch%d", ch, ch).c_str(), a.value("channels", 32) == ch)) a["channels"] = ch;
      ImGui::SameLine();
    }
    ImGui::NewLine();
    attr_float("量程 m", "range", 10, 300, "%.0f");
    float pps = a.value("points_per_second", 600000.0f) / 1e4f;
    ui::Row("每秒点数（万）", "常见：16 线 30 万，32 线 60-70 万，64 线 130 万，128 线 240 万", fs * 12);
    if (ImGui::SliderFloat("##pps", &pps, 5, 500, "%.0f")) a["points_per_second"] = static_cast<int>(pps * 1e4f);
    attr_float("上视场 °", "upper_fov", -30, 30, "%.1f");
    attr_float("下视场 °", "lower_fov", -60, 0, "%.1f");
    ui::DimWrapped("每帧自动扫完整一圈；点云按 CarSim 方向给出和保存（x 前、y 左、z 上）");
  } else if (t == "radar") {
    attr_float("水平视场 °", "horizontal_fov", 5, 180, "%.0f");
    attr_float("垂直视场 °", "vertical_fov", 1, 60, "%.0f");
    attr_float("量程 m", "range", 10, 300, "%.0f");
    attr_int("每秒点数", "points_per_second", 100);
  }
  ImGui::EndDisabled();
  ImGui::Dummy(ImVec2(0, 2));
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui::LabelWidth());
  if (t == "rgb") {
    ImGui::BeginDisabled(world_.value("ego_id", 0) == 0);
    if (ui::Button(ICON_FA_EYE, "预览这个相机", ui::Kind::Primary)) {
      json m = {{"x", s["x"]}, {"y", s["y"]}, {"z", s["z"]}, {"pitch", s["pitch"]}, {"yaw", s["yaw"]}, {"roll", s["roll"]},
                {"frame", "carsim"}, {"ref", cfg_["sync"].value("reference_point", json("front_axle"))}};
      StartView(m, a.value("fov", 90.0f));
      view_rig_sensor_ = s.value("name", std::string());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
  }
  ImGui::BeginDisabled(Running());
  if (ui::Button(ICON_FA_PASTE, "复制")) {
    json cp = s;
    cp["name"] = s.value("name", std::string()) + "_copy";
    cp["y"] = s.value("y", 0.0) + 0.3;
    sensors.push_back(cp);
    rig_sel_ = static_cast<int>(sensors.size()) - 1;
  }
  ImGui::SameLine();
  if (ui::Button(ICON_FA_TRASH, "删除", ui::Kind::Danger)) {
    sensors.erase(sensors.begin() + rig_sel_);
    rig_sel_ = std::min(rig_sel_, static_cast<int>(sensors.size()) - 1);
  }
  ImGui::EndDisabled();
  ui::EndCard();
}

// --------------------------------------------------------------------------
void App::DrawRigEstimate(bool compact) {
  const ui::Palette& p = ui::Colors();
  const float fs = ImGui::GetFontSize();
  const json& sensors = RigSensors();
  const json& c = cfg_.contains("collect") ? cfg_["collect"] : json::object();
  const std::string imf = c.value("image_format", std::string("jpg"));
  double per_frame = 20000;  // labels + ego json
  int n_on = 0;
  for (const json& s : sensors)
    if (s.value("enabled", true)) {
      per_frame += BytesPerFrame(s, imf, cfg_["sync"].value("frame_dt", 0.02));
      ++n_on;
    }
  const double dt = cfg_.contains("sync") ? cfg_["sync"].value("frame_dt", 0.1) : 0.1;
  const double sp = c.value("sample_period", 0.0);
  const int every = sp > 0 ? std::max(1, static_cast<int>(std::lround(sp / std::max(1e-3, dt)))) : std::max(1, c.value("capture_every", 1));
  const double hz = 1.0 / (std::max(1e-3, dt) * every);
  const double mbps = per_frame * hz / 1e6;
  const int mf = c.value("max_frames", 0);
  const double ms = c.value("max_seconds", 0.0), mg = c.value("max_gb", 0.0);
  double total_gb = -1;
  if (mf > 0) total_gb = per_frame * mf / 1e9;
  else if (ms > 0) total_gb = per_frame * hz * ms / 1e9;
  if (mg > 0 && (total_gb < 0 || total_gb > mg)) total_gb = mg;
  const double free_gb = disk_.value("free_gb", -1.0);

  ui::BeginCard(ICON_FA_CHART_LINE, "数据量估算", compact ? "est_c" : "est_f");
  const float tile_w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 3) / 4.0f;
  ui::KpiTile("每帧", Fmt("%.1f", per_frame / 1e6).c_str(), "MB", tile_w);
  ImGui::SameLine();
  ui::KpiTile("写盘速率", Fmt("%.1f", mbps).c_str(), "MB/s", tile_w);
  ImGui::SameLine();
  ui::KpiTile("每小时", Fmt("%.0f", mbps * 3.6).c_str(), "GB", tile_w);
  ImGui::SameLine();
  ui::KpiTile("本次最多", total_gb >= 0 ? Fmt("%.2f", total_gb).c_str() : "不限", total_gb >= 0 ? "GB" : "", tile_w);
  ImGui::TextColored(p.text_dim, "%d 个传感器 · %.0f Hz · %s%s", n_on, hz, imf == "jpg" ? "JPG" : "PNG",
                     " · 数值按 CARLA 城市场景实测比例估算，采集时显示真实速率");
  if (free_gb >= 0) {
    const float used = total_gb >= 0 ? static_cast<float>(total_gb / std::max(1e-6, free_gb)) : 1.0f;
    const ImVec4 col = total_gb < 0 ? p.danger : (total_gb > free_gb - 10 ? p.danger : used > 0.5f ? p.warning : p.success);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
    ImGui::ProgressBar(std::min(1.0f, used), ImVec2(-1, fs * 0.9f), "");
    ImGui::PopStyleColor();
    if (total_gb < 0)
      ImGui::TextColored(p.danger, ICON_FA_TRIANGLE_EXCLAMATION "  没有停止条件，会一直写到磁盘满，后端会拒绝开始");
    else if (total_gb > free_gb - 10)
      ImGui::TextColored(p.danger, ICON_FA_TRIANGLE_EXCLAMATION "  预计 %.1f GB，超过可用空间（剩余 %.0f GB，需保留 10 GB）", total_gb, free_gb);
    else
      ImGui::TextColored(p.text_dim, "预计占用 %.2f GB / 剩余 %.0f GB（%s）", total_gb, free_gb, disk_.value("path", std::string()).c_str());
  }
  ui::EndCard();
}
