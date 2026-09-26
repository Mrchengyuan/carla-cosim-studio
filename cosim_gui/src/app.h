// CARLA CoSim Studio: CarSim-style GUI platform for CARLA (+ CarSim co-sim).
// Every CARLA / CarSim operation goes through backend_server.py.
#pragma once

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "backend_client.h"
#include "imgui.h"
#include "platform.h"

struct TourStep;

enum Panel {
  kPanelConnect,
  kPanelWorld,
  kPanelTraffic,
  kPanelActors,
  kPanelVehicle,
  kPanelRig,
  kPanelDrive,
  kPanelCoSim,
  kPanelCollect,
  kPanelRecorder,
  kPanelView,
  kPanelDataset,
  kPanelScene,
  kPanelCount
};

class App {
 public:
  App();
  ~App();

  void Init(int argc, char** argv);
  void Frame();
  // Between the platform backend's NewFrame (which reads the real cursor) and
  // ImGui::NewFrame: tour clicks queued here win over the cursor on every OS.
  void BeforeNewFrame() { TourClick(); }
  void AfterRender(int fb_w, int fb_h);
  bool WantsQuit() const { return quit_; }
  std::string FontPathOverride() const { return font_path_; }
  bool DarkTheme() const { return dark_; }
  bool ThemeChanged() { bool c = theme_changed_; theme_changed_ = false; return c; }

 private:
  // ---------------------------------------------------------- layout (ui_layout.cpp)
  void DrawMenuBar();
  void DrawAbout();
  void HandleShortcuts();
  void DrawToolbar();
  void DrawNav();
  void DrawProperties();
  void DrawViewport(float w, float h);
  void DrawHud(ImVec2 bottom_left);
  void DrawMinimap(ImVec2 top_left, float size);
  void DrawDock(float w, float h);
  void DrawPlots();
  void DrawVehicleState();
  void DrawSceneTab();
  void DrawSceneBev(const json& sc, ImVec2 size);
  void DrawLogList(int warns, int errors);
  void DrawStatusBar();

  // ---------------------------------------------------------- panels (panels.cpp)
  void DrawPanelConnect();
  void DrawPanelWorld();
  void DrawPanelTraffic();
  void DrawPanelActors();
  void DrawPanelVehicle();
  void DrawPanelDrive();
  void DrawPanelScene();
  void DrawPanelCoSim();
  void DrawPanelCollect();
  void DrawPanelRecorder();
  void DrawPanelView();
  void DrawViewImage(float max_w, float max_h);

  // ---------------------------------------------------------- rig editor (rig_editor.cpp)
  void DrawPanelRig();
  void DrawRigTopView(float w, float h);
  void DrawRigSideView(float w, float h);
  void DrawRigProperties();
  void DrawRigEstimate(bool compact);
  json& RigSensors();
  void LoadRigPreset(const std::string& preset);
  void AddRigSensor(const std::string& type);

  // ---------------------------------------------------------- actions (app.cpp)
  void StartBackend();
  void StopBackend();
  void RestartBackend();
  void ConnectBackend(bool quiet = false);  // quiet: a retry while it starts, no error
  void ConnectCarla(bool recover = false);
  void RefreshWorld();
  void RefreshAfterMapChange();
  void LoadMap(const std::string& name);
  void ApplyWeatherPreset(const std::string& preset);
  void ApplyWeatherParams();
  void ApplyWorldSettings();
  void RefreshVehicles();
  void FetchVehicleSpecs(bool all);
  void RefreshSpawnPoints();
  void SpawnEgo();
  void DestroyEgo();
  void SetSpectator(const std::string& mode);
  void SpawnTraffic();
  void ClearTraffic();
  void StartRun();
  void RunCommand(const std::string& cmd);
  void RefreshActors();
  void RefreshDisk();
  void StartView(const json& mount = json(), float fov = 90.0f);  // main camera; mount = rig camera preview
  void SendViews();                        // (re)start every visible viewport pane
  json PaneSpec(const std::string& source, int w, int h) const;
  std::vector<std::pair<std::string, std::string>> ViewSources();  // (source id, label) for pane pickers
  void DrawPane(int i, ImVec2 pos, ImVec2 size);

  // ---------------------------------------------------------- dataset browser (dataset_browser.cpp)
  void DrawPanelDataset();
  void DrawDatasetViewport(float w, float h);
  void DatasetRefresh();
  void DatasetOpen(const std::string& root);
  void DatasetRequestFrame();
  void DatasetTick();
  int DatasetFrameCount() const;
  int DatasetFrameNumber() const;
  void UploadViewTexture();
  void UpdateKeyboardDriving();
  // Relative paths the user types are relative to the bridge directory, like
  // every other path of a run (controller file, logs, datasets).
  std::string UserPath(const std::string& path) const;
  void LoadConfig(const std::string& path);
  void ConformConfig();
  void SaveConfig(const std::string& path);
  void SavePrefs();
  void OnEvent(const json& ev);
  void Log(const std::string& msg, const std::string& level = "info");
  void Call(const std::string& cmd, json args, std::function<void(const json&)> on_ok,
            const std::string& busy_text = "");
  const json* SelectedVehicleSpec() const;
  bool Running() const { return run_state_ == "running" || run_state_ == "paused"; }
  void SetWorld(const json& r);  // world_info reply

  // ---------------------------------------------------------- state
  BackendClient be_;
  plat::Process backend_proc_;
  json prefs_;
  json prefs_file_;          // prefs as read from the file (before command-line overrides)
  json prefs_cli_;           // values the command line set: saved only if the user changed them
  json cfg_defaults_;        // the backend's default config (types to repair loaded configs with)
  std::string prefs_path_;
  json cfg_;                // run config, same schema as settings.py
  std::string cfg_path_;
  bool quit_ = false;
  bool auto_connect_ = false;  // --auto-connect: connect to CARLA once the backend is up
  bool recover_connect_ = false;  // after RestartBackend: reconnect and clear what the old one left
  std::string busy_task_;         // what the backend worker has been busy with ("busy" heartbeat)
  double busy_secs_ = 0, busy_seen_ = 0;
  bool busy_carla_gone_ = false;  // ... and CARLA does not listen any more
  std::string backend_problem_;   // backend hung or died: viewport banner with a restart button
  bool dark_ = true, theme_changed_ = false;
  int panel_ = kPanelConnect;
  std::string busy_;
  std::string font_path_;

  bool carla_connected_ = false;
  json server_info_ = json::object();
  json world_ = json::object();
  std::vector<std::string> maps_, weathers_;
  std::string map_choice_, weather_choice_ = "ClearNoon";
  json weather_edit_ = json::object();
  json vehicles_ = json::array();
  int vehicle_sel_ = -1;
  std::string vehicle_filter_;
  json spawn_points_ = json::array();
  json actors_ = json::array();
  std::string actor_filter_ = "*";
  float ego_color_[3] = {0.12f, 0.20f, 0.55f};
  bool ego_custom_color_ = false;
  bool autopilot_ = false;
  json disk_ = json::object();

  int traffic_vehicles_ = 30, traffic_walkers_ = 20, traffic_seed_ = 0;
  bool traffic_safe_ = true;
  json traffic_count_ = json::object();

  // rig editor
  json rig_presets_ = json::array();
  std::string rig_preset_choice_ = "nuscenes";
  int rig_sel_ = -1;

  // recorder
  std::string rec_file_ = "cosim_record.log";
  bool recording_ = false;
  float replay_start_ = 0.0f, replay_duration_ = 0.0f;
  int replay_follow_ = 0;
  std::string rec_info_;

  // export-variable editor
  std::string new_export_, paste_exports_;

  // run state + telemetry
  std::string run_state_ = "stopped";
  json run_info_ = json::object();
  json last_tel_ = json::object();
  json last_scene_;                  // what the control algorithm got last (kept after the run ends)
  std::string scene_hover_;          // object id under the mouse in the scene table
  bool scene_moving_only_ = false;   // scene tab: hide the parked cars of the map
  json collect_stats_ = json::object();
  static constexpr int kHist = 900;
  std::vector<float> h_t_, h_speed_, h_steer_fl_, h_steer_fr_, h_rt_, h_susp_[4], h_thr_, h_brk_;

  // keyboard driving
  float kb_throttle_ = 0, kb_brake_ = 0, kb_steer_ = 0;
  double kb_last_send_ = 0;

  struct LogLine { std::string level, time, text; };
  std::deque<LogLine> log_;
  bool log_open_ = true, log_scroll_ = false;  // log_open_: bottom dock visible
  int log_errors_ = 0;
  int log_filter_ = 0;  // 0 all, 1 warnings + errors, 2 errors

  // window layout (sizes in pixels, set from the font size on the first frame)
  float nav_w_ = 0, mon_w_ = 0, console_h_ = 0;
  bool monitor_open_ = true, about_open_ = false;  // monitor_open_: properties panel visible
  int dock_tab_select_ = -1;                       // >= 0: select this dock tab next frame
  bool view_auto_ = true;                          // open the viewport camera when an ego appears
  int view_auto_ego_ = 0;
  std::vector<float> trail_x_, trail_y_;           // ego path of the current run (minimap)
  std::string run_note_, run_note_level_;          // why the last run ended (viewport banner)
  int run_note_ego_ = 0;                           // the ego that banner is about
  bool nav_collapsed_[8] = {};

  // live view
  bool view_on_ = false;
  std::string view_mode_ = "chase";
  std::string view_rig_sensor_;  // non-empty: previewing a rig camera
  int view_res_ = 2;  // 960x540: the viewport is large
  unsigned int view_tex_ = 0;
  int view_w_ = 0, view_h_ = 0, view_frames_ = 0;
  int views_pending_ = 0;  // views_set calls not answered yet
  std::vector<unsigned char> view_pixels_;
  bool view_dirty_ = false;
  json view_rig_mount_;          // mount of the rig camera shown in the main pane
  float view_rig_fov_ = 90.0f;
  // Extra viewport panes (index 1..3; the main pane is view_* above).
  struct ViewPane {
    std::string source;          // cam:chase … | semantic | depth | instance | lidar | radar | rig:<name>
    unsigned int tex = 0;
    int w = 0, h = 0, frames = 0;
    std::vector<unsigned char> px;
    bool dirty = false;
  };
  ViewPane panes_[4] = {{}, {"semantic"}, {"lidar"}, {"depth"}};
  int view_layout_ = 0;          // 0 single, 1 one large + three small, 2 grid 2x2

  // dataset browser
  json ds_sessions_;             // null until first listed
  json ds_info_ = json::object(), ds_objects_ = json::array(), ds_frame_info_ = json::object();
  json ds_export_result_ = json::object();
  std::string ds_root_, ds_left_, ds_right_, ds_export_cam_, ds_export_lidar_, ds_out_, ds_delete_;
  int ds_idx_ = 0, ds_pending_ = 0, ds_fps_ = 5, ds_fmt_ = 0, ds_min_pts_ = 1, ds_export_done_ = 0, ds_export_total_ = 0;
  bool ds_boxes_ = true, ds_play_ = false, ds_exporting_ = false, ds_dirty_ = false;
  bool props_scroll_end_ = false;  // tour: scroll the properties panel to its end next frame
  double ds_last_step_ = 0;
  ViewPane ds_panes_[2];

  // --tour automation (screenshots of every panel for testing)
  std::string tour_dir_;
  std::vector<TourStep>* tour_ = nullptr;
  size_t tour_i_ = 0;
  int tour_wait_ = 0;
  std::string shot_name_;
  int frame_ = 0;
  void TourTick();
  void BuildTour();
  void BuildHeroTour();           // --hero: candidate screenshots for the README cover
  std::string hero_spawns_;       // --hero-spawns 0,10,20
  void TourClick();                 // feeds a pending tour click to ImGui as real mouse events
  std::string click_target_;
  int click_phase_ = 0;
  int tour_mark_ = 0;               // value remembered by a tour step (e.g. the frame before a single step)

  friend struct UiAccess;
};

// helpers shared by the ui files
namespace appui {
bool InputStr(const char* label, std::string& s, int flags = 0);
bool InputStrMultiline(const char* label, std::string& s, float w, float h);
bool ComboStr(const char* label, std::string& value, const std::vector<std::string>& items,
              const std::vector<std::string>* shown = nullptr);
std::string Fmt(const char* fmt, ...);
bool Base64(const std::string& in, std::vector<unsigned char>& out);
// Element i of a JSON array as a number; `def` when missing, null (the backend
// sends NaN / inf as null) or not a number. Never throws.
inline double NumAt(const json& j, size_t i, double def = 0.0) {
  return j.is_array() && i < j.size() && j[i].is_number() ? j[i].get<double>() : def;
}
}  // namespace appui
