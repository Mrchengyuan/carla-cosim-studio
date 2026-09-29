// CARLA CoSim Studio: CarSim-style GUI platform for CARLA (+ CarSim co-sim).
// Every CARLA / CarSim operation goes through backend_server.py.
#pragma once

#include <deque>
#include <map>
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
  kPanelTestScene,
  kPanelRuns,
  kPanelBatch,
  kPanelCount
};

// The 测试场景 presets (a lane closure list each), shared by the 测试场景 and 批量测试 pages.
struct TestScenePreset {
  const char* id;
  const char* name;
  const char* tip;
  json closures;
};
const std::vector<TestScenePreset>& TestScenePresets();
// The 动态目标 presets (one moving actor each: its settings in "closures"), also for 批量测试.
const std::vector<TestScenePreset>& TestActorPresets();

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
  // quit_ was asked for: the backend is stopped first, with the window still
  // drawing; true once it is gone.
  bool WantsQuit();
  // The window's close button and 文件 → 退出: with unsaved config changes, ask first.
  void AskQuit();
  std::string WindowTitle() const;  // ends in " *" while the config has unsaved changes
  std::string FontPathOverride() const { return font_path_; }
  bool DarkTheme() const { return dark_; }
  bool ThemeChanged() { bool c = theme_changed_; theme_changed_ = false; return c; }

 private:
  // ---------------------------------------------------------- layout (ui_layout.cpp)
  void FrameBody();  // Frame() after polling the backend
  void DrawMenuBar();
  void DrawAbout();
  void DrawConfirmDialogs();  // unsaved changes on quit, "默认" on the CarSim page
  void HandleShortcuts();
  void DrawToolbar();
  void DrawNav();
  void DrawProperties();
  void DrawViewport(float w, float h);
  void DrawHud(ImVec2 bottom_left);
  void DrawMinimap(ImVec2 top_left, float size);
  void DrawPanels(float fs);
  void BuildDefaultLayout(unsigned int dock_id, ImVec2 size, float fs);
  void DrawDrawTab();
  void DrawPlots();
  void DrawVehicleState();
  void DrawSceneTab();
  void DrawSceneBev(const json& sc, ImVec2 size);
  void DrawLogList(int warns, int errors, int algos);
  void DrawStatusBar();
  bool PageEnabled(int panel) const;  // pages that only edit the config or read datasets work without CARLA

  // ---------------------------------------------------------- panels (panels.cpp)
  void DrawPanelConnect();
  void DrawPanelWorld();
  void DrawPanelTraffic();
  void DrawPanelActors();
  void DrawPanelVehicle();
  void DrawPanelDrive();
  void DrawPanelTestScene();
  void DrawPanelRuns();
  void DrawPanelBatch();
  void BatchTick();
  std::vector<json> BatchPlan();
  void DrawCompare();
  void DrawIdent();
  void DrawDisturb();
  void DrawCriteria();  // 通过标准 (批量测试 page, config "criteria")  // 干扰 (驾驶模式 page, run.disturb)  // 车辆参数辨识 card (运行对比 page)
  void RunsRefresh();
  void RunsToggle(const std::string& folder);
  void DrawPanelScene();
  void DrawPanelCoSim();
  void DrawPanelCollect();
  void DrawPanelRecorder();
  void DrawPanelView();
  void DrawViewImage(float max_w, float max_h);
  void BackendPathStatus(const std::string& path);  // check / cross under a path the backend reads
  void EditAlgoPath(json& ctl);
  void DrawAlgoParams(const json& ctl);
  void DrawWorldFixed();                        // 控制算法 file field with its “浏览…”
  void AlgoBrowse(const std::string& path);            // remote: a folder of the server (controller_browse)
  void DrawAlgoBrowser(json& ctl);                     // remote: the server's algorithm files to pick from

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
  void StopBackendPoll();  // every frame while StopBackend waits for the backend to exit
  void RestartBackend();
  void ConnectBackend(bool quiet = false);  // quiet: a retry while it starts, no error
  void LoadBackendDefaults();                // default config and rig presets of a backend just connected
  void ConnectCarla(bool recover = false);
  void RefreshWorld();
  void RefreshAfterMapChange();
  void LoadMap(const std::string& name, std::function<void()> then = nullptr);  // then: once it is loaded
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
  // over: settings merged over the page's config for this run (批量测试); done: after the backend's answer.
  void StartRun(const json& over = json(), std::function<void(bool, const json&, const std::string&)> done = nullptr);
  void RunCommand(const std::string& cmd);
  void RefreshActors();
  void RefreshDisk();
  void StartView(const json& mount = json(), float fov = 90.0f);  // main camera; mount = rig camera preview
  void SendViews();                        // (re)start every visible viewport pane
  json PaneSpec(const std::string& source, int w, int h) const;
  json RigMount(const json& s) const;      // a rig sensor's mount for a view, in the rig's frame
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
  bool ConfigDirty() const { return cfg_ != saved_cfg_; }
  void SavePrefs();
  void OnEvent(const json& ev);
  void Log(const std::string& msg, const std::string& level = "info");
  void Call(const std::string& cmd, json args, std::function<void(const json&)> on_ok,
            const std::string& busy_text = "");
  const json* SelectedVehicleSpec() const;
  bool Running() const { return run_state_ == "running" || run_state_ == "paused"; }
  // The backend runs on a server, reached through an SSH tunnel (prefs "remote_backend"):
  // never started or stopped here; CarSim runs in the CarSim service on this computer.
  bool Remote() const { return prefs_.value("remote_backend", false); }
  // The rig's mounts are still an older config's (CARLA frame: car centre, y right).
  bool RigLegacy() const {
    return cfg_.contains("rig") && cfg_["rig"].is_object() && cfg_["rig"].value("frame", std::string("carsim")) == "carla";
  }
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
  json saved_cfg_;          // cfg_ as last loaded or saved: unsaved changes = cfg_ != saved_cfg_
  bool quit_ = false;
  bool quit_ask_ = false, reset_ask_ = false;  // open that confirmation dialog next frame
  std::string draw_error_;  // what the last frame cut short by a bad value threw (logged once)
  bool auto_connect_ = false;  // --auto-connect: connect to CARLA once the backend is up
  bool recover_connect_ = false;  // after RestartBackend: reconnect and clear what the old one left
  bool backend_rejected_ = false;  // the backend on our port serves another GUI window: stop reconnecting
  json carsim_service_ = json::object();  // remote: the CarSim service on this computer (backend's view)
  bool hello_ok_ = false;          // the backend answered this connection's hello (remote: an SSH tunnel
                                   // accepts a connection even while no backend listens behind it)
  json server_paths_ = json::object();  // remote: "path_status" answers, path -> {asked, pending, status}
  // 控制算法 “浏览…” with the backend on a server: the folder shown (controller_browse answer), the file picked
  bool algo_browser_open_ = false, algo_browse_pending_ = false;
  json algo_browse_ = json::object(), algo_sel_;
  std::string algo_browse_err_, algo_sel_entry_;
  bool rig_converting_ = false;   // an old config's rig (CARLA frame) is being converted by the backend,
                                  // or that failed (retried on the next CARLA connection / config load)
  std::string busy_task_;         // what the backend worker has been busy with ("busy" heartbeat)
  double busy_secs_ = 0, busy_seen_ = 0;
  bool busy_carla_gone_ = false;  // ... and CARLA does not listen any more
  std::string busy_where_;        // task "control" / "finish": the line the user's control() / finish() is at
  std::string backend_problem_;   // backend hung or died: viewport banner with a restart button
  double stop_since_ = -1;        // StopBackend asked the backend to exit at this time (< 0: not stopping)
  bool stop_asked_ = false, stop_termed_ = false;  // ... over the connection / SIGTERM sent
  bool dark_ = true, theme_changed_ = false;
  int panel_ = kPanelConnect;
  std::string busy_;
  bool starting_ = false;         // cosim_start sent, reply not back yet
  std::string font_path_;

  bool carla_connected_ = false;
  json server_info_ = json::object();
  json world_ = json::object();
  std::vector<std::string> maps_, weathers_;
  std::string map_choice_, weather_choice_ = "ClearNoon";
  json weather_edit_ = json::object();
  bool weather_dirty_ = false;    // weather_edit_ has slider changes not applied yet
  json world_edit_;               // 仿真设置 changes not applied yet (null: none)
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
  json draw_ = json::array();        // the algorithm's lines (self.draw), ego frame: the 轨迹 tab
  // 运行对比: the run records of a record dir, up to 4 picked, their time series; the 对比 panel.
  // 算法参数: the algorithm file's upper-case constants (the backend's controller_params), for which file.
  json algo_params_ = json::object();
  std::string algo_params_path_;
  bool algo_params_pending_ = false;
  // 批量测试: what to run (scenario presets x spawn points x values of one algorithm constant),
  // the runs one after the other, their results and the report.
  bool batch_scn_[9] = {true, false, false, false, false, false, false, false, false};  // 不开封道, closures, moving actors
  std::string batch_spawns_, batch_param_, batch_values_;
  double batch_duration_ = 40.0;
  std::vector<json> batch_items_, batch_results_;
  int batch_i_ = -1;
  bool batch_running_ = false, batch_stop_ = false, batch_wait_ = false;
  std::string batch_dir_;
  json batch_report_ = json::object();
  std::string runs_path_;
  json runs_list_ = json::object();
  std::vector<std::string> runs_sel_;
  std::map<std::string, json> runs_series_;
  int runs_pending_ = 0;
  bool compare_open_ = false, compare_focus_ = false;
  std::string compare_dbg_;          // the self.debug value shown in the 对比 panel
  // Its timeline: the time shown on every plot, played at a speed; 在 CARLA 里看这一刻 places the
  // ego where run A had it then (the backend's replay_pose, a request at a time).
  double compare_t_ = 0.0, compare_sent_t_ = -1.0, compare_sent_at_ = 0.0;
  float compare_speed_ = 1.0f;
  bool compare_play_ = false, compare_carla_ = false, compare_replay_pending_ = false;
  // 车辆参数辨识 (运行对比 page): the last result (for run ident_folder_), a request out.
  json ident_result_;
  // 检查 .sim (CarSim 动力学 page): the backend's check_sim result, a request out.
  json simcheck_;
  bool simcheck_pending_ = false;
  // 运行对比: run A's 输出 (output.txt) in a window.
  json run_out_;
  std::string run_out_folder_;
  bool run_out_open_ = false, run_out_pending_ = false, run_out_algo_only_ = false;
  void DrawRunOutput();
  // 文件 → 从模板新建 (the backend's templates_list) / 最近打开 (prefs "recent_configs").
  json templates_;
  bool templates_pending_ = false;
  std::string template_ask_;  // a template's id: its confirmation dialog next frame
  void AddRecent(const std::string& path);
  void DrawRecentMenu();
  void ApplyTemplate(const std::string& id);
  void UseScenarioStart();    // the map's 测试场景 start as the spawn point (world_ scenario_start)
  void UseTown04Start();      // that, loading Town04 first when this map has none
  std::string ident_folder_;
  bool ident_pending_ = false;
  int draw_mag_ = 0;                 // 轨迹 tab's lateral magnification: 0 = fit the lane
  json collect_stats_ = json::object();
  static constexpr int kHist = 900;
  // h_thr_, h_brk_, h_u3_: the first three values control() returned (imports 1-3);
  // h_steer_*: front wheel angles, + = left (CarSim's sign).
  std::vector<float> h_t_, h_speed_, h_steer_fl_, h_steer_fr_, h_rt_, h_susp_[4], h_thr_, h_brk_, h_u3_;
  // The algorithm's self.debug values over time (a series per name, in the order they came; NaN: not given).
  std::vector<std::pair<std::string, std::vector<float>>> h_dbg_;
  // The .sim's imports as the last telemetry had them: [油门, 制动, 方向盘] (the
  // default 3, and always with CARLA dynamics) or else shown as 导入 1 ... n.
  int n_imports_ = 3;
  bool imports_named_ = true;

  // keyboard driving
  float kb_throttle_ = 0, kb_brake_ = 0, kb_steer_ = 0;
  double kb_last_send_ = 0;

  struct LogLine { std::string level, time, text; };
  std::deque<LogLine> log_;
  bool log_open_ = true, log_scroll_ = false;  // log_open_: bottom dock visible
  int log_errors_ = 0;
  int log_filter_ = 0;  // 0 all, 1 warnings + errors, 2 errors, 3 the control algorithm's own output

  // window layout (sizes in pixels, set from the font size on the first frame)
  // Dockable panels (Dear ImGui docking): each region is a window the user can move,
  // stack as tabs, float inside the main window or close (视图 menu reopens it).
  bool nav_open_ = true, view_open_ = true;
  bool bottom_open_[5] = {true, true, true, true, true};  // 曲线, 车辆状态, 输出, 场景, 轨迹 (dock_tab_select_ order)
  unsigned int dock_id_ = 0;                               // the main dock space
  bool layout_checked_ = false, layout_reset_ = false;    // layout_reset_: build the default layout next frame
  std::string layout_ini_;                                // where the layout is kept (none in a tour)
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
  ImVec2 drag_to_{-1.0f, -1.0f};  // >= 0: the click on click_target_ is a drag to here (a dock tab)
  ImVec2 drag_from_{-1.0f, -1.0f}; // where that drag started (the tab moves with the mouse); < 0: not yet
  int click_phase_ = 0;
  ImVec2 click_last_{-1.0f, -1.0f};  // target position last frame
  int tour_mark_ = 0;               // value remembered by a tour step (e.g. the frame before a single step)
  json tour_kept_;                  // what the tour's config steps change, put back after them

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
// A frame's pixels (RGB, w x h) from its raw "rgb" or its "jpeg" (remote GUI).
bool FramePixels(const nlohmann::json& ev, std::vector<unsigned char>& px, int& w, int& h);
// Element i of a JSON array as a number; `def` when missing, null (the backend
// sends NaN / inf as null) or not a number. Never throws.
inline double NumAt(const json& j, size_t i, double def = 0.0) {
  return j.is_array() && i < j.size() && j[i].is_number() ? j[i].get<double>() : def;
}
}  // namespace appui
