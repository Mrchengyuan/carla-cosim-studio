// Remote launcher (启动远程仿真.exe): the Windows laptop's side of remote mode
// (docs/远程使用指南.md). One window that checks this computer, opens the SSH
// connection to the server (which starts CARLA and the backend there), runs
// the CarSim service here and opens the GUI; closing the GUI ends it all.
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "child_process.h"

class Launcher {
 public:
  Launcher();
  ~Launcher();
  void Init(const std::vector<std::string>& args);
  void BeforeNewFrame();                // --tour: clicks as real mouse events
  void Frame();                         // state machine + UI
  void AfterRender(int fb_w, int fb_h);  // --tour: screenshots
  void AskQuit();                       // the window's close button
  bool WantsQuit() const { return quit_; }
  bool Animating() const;               // something moves: draw often
  bool Dark() const { return dark_; }
  bool ThemeChanged();
  // 2: bring the window to the front (the run ended or failed); 0: nothing.
  int TakeWindowRequest();
  std::string WindowTitle() const;

  enum class St { Idle, Busy, Ok, Warn, Fail };
  enum class Phase { Checking, Idle, Connecting, Starting, Ready, Stopping, Stopped, Failed };
  enum Src { kLauncher, kCloud, kService, kGui };
  enum Lvl { kInfo, kOk, kWarn, kErr };

 private:
  struct Check {
    std::string name, detail, hint;
    St st = St::Idle;
  };
  struct LogLine {
    std::string time;
    int src, lvl;
    std::string text;
  };
  struct Part {  // cloud / CarSim service / GUI, as the step bar shows it
    St st = St::Idle;
    std::string detail;
  };
  using Clock = std::chrono::steady_clock;

  // ---- state machine ----
  void Update();
  void StartChecks();
  void UpdateChecks();
  void FinishChecks();
  void Start();
  void StartSsh();
  void RunService();
  void StartGui();
  void Stop(const std::string& why, bool gui_closed);
  void Fail(const std::string& title, const std::string& hint);
  void OnCloudLine(const std::string& line);
  void OnServiceLine(const std::string& line);
  void OnGuiLine(const std::string& line);
  void SetPhase(Phase p);
  double Since(Clock::time_point t) const;
  bool Running() const;

  // ---- environment ----
  void CheckFiles();
  void CheckSsh();
  void CheckPorts();
  void PrepareKey();
  void FixKey(const std::string& key);
  void StartPythonProbe();
  void InstallNumpy();
  std::string PortOwner(int port, long* pid);
  void LoadSettings();
  void SaveSettings();

  // ---- UI ----
  void DrawHeader();
  void DrawSteps();
  void DrawBanner();
  void DrawChecks();
  void DrawSettings();
  void DrawLog();
  void DrawModals();
  bool Btn(const char* id, const char* icon, const char* text, int kind, float w = 0.0f);
  void Log(int src, int lvl, const std::string& text);

  // ---- --tour ----
  void BuildTour(const std::string& script);
  void TourTick();
  void WriteState();

  // paths
  std::string ssh_cwd_;  // where ssh runs: the folder with its key copy
  std::string dir_, ssh_dir_, key_, known_, service_, gui_exe_, settings_path_, notes_, state_dir_, log_path_;
  // settings (remote_launcher.json next to the program)
  std::string host_, user_, python_setting_;
  int port_ = 0;
  bool dark_ = true, theme_changed_ = false;
  int local_gui_port_ = 57120, local_carsim_port_ = 57121;

  Phase phase_ = Phase::Checking;
  Clock::time_point phase_t_, start_t_, t0_;
  std::string fail_title_, fail_hint_, stop_note_;
  bool auto_start_ = true;
  bool quit_ = false;
  int window_request_ = 0;

  // checks
  std::vector<Check> checks_;  // files, ssh, python, numpy, ports
  std::string ssh_exe_;
  std::vector<std::string> py_argv_;  // the Python that works (e.g. {"C:/Windows/py.exe", "-3"})
  std::vector<std::vector<std::string>> py_candidates_;
  size_t py_try_ = 0;
  std::string py_version_, py_bits_, py_exe_, numpy_version_;
  std::unique_ptr<plat::Child> probe_;
  Clock::time_point probe_t_;
  std::vector<std::string> probe_out_;
  std::unique_ptr<plat::Child> pip_;
  std::vector<long> busy_pids_;        // ssh.exe holding our ports (a leftover tunnel)
  bool checks_ok_ = false;

  // children
  std::unique_ptr<plat::Child> ssh_, service_proc_, gui_proc_;
  Part cloud_, svc_, gui_;
  bool tunnel_up_ = false, server_ready_ = false, ready_once_ = false, service_started_ = false;
  int reconnects_ = 0;
  Clock::time_point reconnect_at_, next_probe_t_, stop_deadline_;
  bool reconnect_pending_ = false;
  std::string ssh_error_;   // the last ssh / server error line, mapped to Chinese
  bool ssh_fatal_ = false;  // ... one that reconnecting cannot help (key, host key, port)
  std::string server_name_;
  bool stopping_gui_closed_ = false;
  std::vector<std::string> gui_extra_;

  // log
  std::vector<LogLine> log_;
  int log_filter_ = -1;  // -1 all, else a Src
  bool log_follow_ = true;
  size_t log_seen_ = 0;

  // modals
  bool ask_quit_ = false, ask_stop_ = false;

  // tour
  struct TourStep {
    std::function<void()> action;
    std::function<bool()> ready;
    std::string shot;
  };
  std::string tour_dir_, shot_name_, click_target_;
  std::vector<TourStep> tour_;
  size_t tour_i_ = 0;
  bool tour_started_ = false;
  int tour_since_ready_ = 0, tour_frames_ = 0, click_phase_ = 0;
  float click_last_x_ = 0, click_last_y_ = 0;
  int frame_ = 0;
};
