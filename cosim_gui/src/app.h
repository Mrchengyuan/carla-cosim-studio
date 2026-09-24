// CARLA CoSim Studio: CarSim-style GUI platform for CARLA (+ CarSim co-sim).
// Every CARLA / CarSim operation goes through backend_server.py.
#pragma once

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "backend_client.h"
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
  kPanelCount
};

class App {
 public:
  App();
  ~App();

  void Init(int argc, char** argv);
  void Frame();
  void AfterRender(int fb_w, int fb_h);
  bool WantsQuit() const { return quit_; }
  std::string FontPathOverride() const { return font_path_; }
  bool DarkTheme() const { return dark_; }
  bool ThemeChanged() { bool c = theme_changed_; theme_changed_ = false; return c; }

 private:
  // ---------------------------------------------------------- layout (ui_layout.cpp)
  void DrawToolbar();
  void DrawNav();
  void DrawMonitor();
  void DrawStatusBar();
  void DrawLogDrawer();

  // ---------------------------------------------------------- panels (panels.cpp)
  void DrawPanelConnect();
  void DrawPanelWorld();
  void DrawPanelTraffic();
  void DrawPanelActors();
  void DrawPanelVehicle();
  void DrawPanelDrive();
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
  void ConnectBackend();
  void ConnectCarla();
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
  void StartView(const json& mount = json(), float fov = 90.0f);
  void UploadViewTexture();
  void UpdateKeyboardDriving();
  void LoadConfig(const std::string& path);
  void SaveConfig(const std::string& path);
  void SavePrefs();
  void OnEvent(const json& ev);
  void Log(const std::string& msg, const std::string& level = "info");
  void Call(const std::string& cmd, json args, std::function<void(const json&)> on_ok,
            const std::string& busy_text = "");
  const json* SelectedVehicleSpec() const;
  bool Running() const { return run_state_ == "running" || run_state_ == "paused"; }

  // ---------------------------------------------------------- state
  BackendClient be_;
  plat::Process backend_proc_;
  json prefs_;
  std::string prefs_path_;
  json cfg_;                // run config, same schema as settings.py
  std::string cfg_path_;
  bool quit_ = false;
  bool auto_connect_ = false;  // --auto-connect: connect to CARLA once the backend is up
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
  json collect_stats_ = json::object();
  static constexpr int kHist = 900;
  std::vector<float> h_t_, h_speed_, h_steer_fl_, h_steer_fr_, h_rt_, h_susp_[4], h_thr_, h_brk_;

  // keyboard driving
  float kb_throttle_ = 0, kb_brake_ = 0, kb_steer_ = 0;
  double kb_last_send_ = 0;

  struct LogLine { std::string level, text; };
  std::deque<LogLine> log_;
  bool log_open_ = false, log_scroll_ = false;
  int log_errors_ = 0;

  // live view
  bool view_on_ = false;
  std::string view_mode_ = "chase";
  std::string view_rig_sensor_;  // non-empty: previewing a rig camera
  int view_res_ = 1;
  unsigned int view_tex_ = 0;
  int view_w_ = 0, view_h_ = 0, view_frames_ = 0;
  std::vector<unsigned char> view_pixels_;
  bool view_dirty_ = false;

  // --tour automation (screenshots of every panel for testing)
  std::string tour_dir_;
  std::vector<TourStep>* tour_ = nullptr;
  size_t tour_i_ = 0;
  int tour_wait_ = 0;
  std::string shot_name_;
  int frame_ = 0;
  void TourTick();
  void BuildTour();

  friend struct UiAccess;
};

// helpers shared by the ui files
namespace appui {
bool InputStr(const char* label, std::string& s, int flags = 0);
bool InputStrMultiline(const char* label, std::string& s, float w, float h);
bool ComboStr(const char* label, std::string& value, const std::vector<std::string>& items,
              const std::vector<std::string>* shown = nullptr);
std::string Fmt(const char* fmt, ...);
}  // namespace appui
