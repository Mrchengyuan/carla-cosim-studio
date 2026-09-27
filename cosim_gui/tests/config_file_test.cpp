// Checks src/config_file.cpp: what the GUI does to the run config when it
// loads, repairs and saves it. carsim_carla_bridge/tests/test_offline_config.py
// builds and runs it with the backend's default config; by hand, in cosim_gui/:
//   c++ -std=c++17 -Isrc -Ithird_party/json tests/config_file_test.cpp src/config_file.cpp -o /tmp/config_file_test
//   /tmp/config_file_test <default_config.json> <empty scratch directory>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "config_file.h"

namespace fs = std::filesystem;

static int g_pass = 0, g_fail = 0;

static void Check(bool ok, const std::string& what, const json& got = json()) {
  std::printf("%s  %s", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok && !got.is_null()) std::printf("  (got %s)", got.dump().c_str());
  std::printf("\n");
  ++(ok ? g_pass : g_fail);
}

static std::string Read(const fs::path& p) {
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static bool NoTmpLeft(const fs::path& dir) {
  for (const auto& e : fs::directory_iterator(dir))
    if (e.path().extension() == ".tmp") return false;
  return true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: config_file_test <default_config.json> <empty scratch directory>\n");
    return 2;
  }
  std::ifstream df(fs::u8path(argv[1]));
  const json defaults = json::parse(df, nullptr, false);
  if (!defaults.is_object()) {
    std::fprintf(stderr, "cannot read %s\n", argv[1]);
    return 2;
  }
  const fs::path dir = fs::u8path(argv[2]);

  // The backend's defaults need no repair; the GUI's driver field is filled from run.driver.
  {
    json cfg = defaults;
    const int fixed = cfgfile::ConformConfig(cfg, defaults);
    Check(fixed == 0 && cfg["drive"]["cosim_driver"] == defaults["run"]["driver"],
          "defaults: nothing to repair, drive.cosim_driver = run.driver", fixed);
  }

  // What the CarSim page stores: an interface forced either way, a reference point.
  for (const json& ext : {json(true), json(false), json("auto")}) {
    json cfg = defaults;
    cfg["sync"]["use_external_api"] = ext;
    cfg["sync"]["reference_point"] = {1.2, 0.0, 0.3};
    const int fixed = cfgfile::ConformConfig(cfg, defaults);
    Check(fixed == 0 && cfg["sync"]["use_external_api"] == ext && cfg["sync"]["reference_point"] == json({1.2, 0.0, 0.3}),
          "sync.use_external_api " + ext.dump() + " and a reference point [x, y, z] are kept, no warning", cfg["sync"]);
  }

  // A hand-edited file: numbers as text, 0 / 1 for bools, wrong types, dead fields.
  {
    json cfg = defaults;
    cfg["sync"]["frame_dt"] = "0.05";
    cfg["collect"]["max_frames"] = "5000";
    cfg["collect"]["enabled"] = 1;
    cfg["collect"]["labels"] = "yes";
    cfg["sync"]["use_external_api"] = 3;
    cfg["sync"]["reference_point"] = {1.0, 2.0};
    cfg["scene"]["objects"] = {"id", 5, "dist"};
    cfg["carla"]["map"] = "Town05";
    cfg["carla"]["weather"] = "ClearNoon";
    cfg["rig"]["sensors"] = {7,
                             {{"name", "cam"}, {"type", "rgb"}, {"x", "1.5"}, {"enabled", "0"},
                              {"attributes", {{"image_size_x", "1600"}, {"fov", "wide"}, {"sensor_tick", "0.1"}}}}};
    const int fixed = cfgfile::ConformConfig(cfg, defaults);
    Check(fixed == 11, "hand-edited file: 11 values repaired", fixed);
    Check(cfg["sync"]["frame_dt"] == 0.05 && cfg["collect"]["max_frames"].is_number_integer() &&
              cfg["collect"]["max_frames"] == 5000,
          "... numbers written as text become numbers", json({cfg["sync"]["frame_dt"], cfg["collect"]["max_frames"]}));
    Check(cfg["collect"]["enabled"] == true && cfg["collect"]["labels"] == defaults["collect"]["labels"],
          "... 1 becomes true, a word that is no bool becomes the default",
          json({cfg["collect"]["enabled"], cfg["collect"]["labels"]}));
    Check(cfg["sync"]["use_external_api"] == "auto" && cfg["sync"]["reference_point"] == defaults["sync"]["reference_point"],
          "... an interface or reference point of no known form becomes the default", cfg["sync"]);
    Check(cfg["scene"]["objects"] == json({"id", "dist"}), "... list entries of the wrong type are dropped", cfg["scene"]["objects"]);
    Check(!cfg["carla"].contains("map") && !cfg["carla"].contains("weather"), "... carla.map / carla.weather dropped", cfg["carla"]);
    const json& s = cfg["rig"]["sensors"];
    Check(s.size() == 1 && s[0]["x"] == 1.5 && s[0]["enabled"] == false && s[0]["attributes"]["image_size_x"] == 1600 &&
              !s[0]["attributes"].contains("fov") && s[0]["attributes"]["sensor_tick"] == 0.1,
          "... sensors: no non-objects, numbers from text, enabled \"0\" = false, a non-numeric fov left to the default", s);
  }

  // drive.cosim_driver: from a file's run.driver when missing or not text.
  {
    json cfg = defaults;
    cfg["run"]["driver"] = "demo";
    cfg["drive"].erase("cosim_driver");
    int fixed = cfgfile::ConformConfig(cfg, defaults);
    Check(fixed == 0 && cfg["drive"]["cosim_driver"] == "demo", "a file without drive.cosim_driver runs its run.driver",
          cfg["drive"]);
    cfg["drive"]["cosim_driver"] = 5;
    fixed = cfgfile::ConformConfig(cfg, defaults);
    Check(fixed == 1 && cfg["drive"]["cosim_driver"] == "demo", "... and one that is not text", cfg["drive"]);
  }

  // Loading: the file over the defaults, not over what was loaded before.
  {
    json current = defaults;
    current["collect"]["enabled"] = true;
    current["collect"]["max_frames"] = 5000;
    current["run"]["controller"]["path"] = "mine.py";
    current["carla"]["vehicle"] = "vehicle.audi.tt";
    current["carla"]["spawn_index"] = 7;
    const json file = {{"run", {{"driver", "custom"}}}, {"sync", {{"frame_dt", 0.05}}}};
    json n = cfgfile::FileOverDefaults(defaults, current, file);
    Check(n["collect"] == defaults["collect"] && n["run"]["controller"] == defaults["run"]["controller"] &&
              n["sync"]["frame_dt"] == 0.05,
          "a partial file: its values over the defaults, not over the config loaded before",
          json({n["collect"], n["run"]["controller"]}));
    Check(n["carla"]["vehicle"] == "vehicle.audi.tt" && n["carla"]["spawn_index"] == 7,
          "... the vehicle and spawn point picked on the 车辆 page stay", n["carla"]);
    n = cfgfile::FileOverDefaults(defaults, current, {{"carla", {{"vehicle", "vehicle.tesla.cybertruck"}, {"spawn_index", 3}}}});
    Check(n["carla"]["vehicle"] == "vehicle.tesla.cybertruck" && n["carla"]["spawn_index"] == 3,
          "... unless the file sets them", n["carla"]);
    n = cfgfile::FileOverDefaults(json(), current, file);
    Check(n["collect"]["enabled"] == true && n["sync"]["frame_dt"] == 0.05,
          "without the defaults yet: over the current config (the defaults come later)", n["collect"]);
  }

  // Saving: the driver in step, the GUI's CARLA server.
  {
    json cfg = defaults;
    cfg["drive"]["cosim_driver"] = "custom";
    cfg["run"]["driver"] = "route";  // left by a run with the route follower
    cfgfile::ForSave(cfg, "carla-box", 3000);
    Check(cfg["run"]["driver"] == "custom" && cfg["carla"]["host"] == "carla-box" && cfg["carla"]["port"] == 3000,
          "a save writes run.driver = drive.cosim_driver and the GUI's host / port",
          json({cfg["run"]["driver"], cfg["carla"]}));
    json bare = {{"run", {{"driver", "demo"}}}};
    cfgfile::SyncRunDriver(bare);
    Check(bare["run"]["driver"] == "demo", "no drive.cosim_driver: run.driver untouched", bare);
  }

  // Save, then load the file again: the same config, nothing to repair (not
  // "unsaved" right after loading), a forced interface included.
  {
    json cfg = defaults;
    cfgfile::ConformConfig(cfg, defaults);
    cfg["sync"]["use_external_api"] = false;
    cfg["sync"]["reference_point"] = {1.4, 0.0, 0.0};
    cfg["rig"]["sensors"] = {{{"name", "front"}, {"type", "rgb"}, {"x", 0.5}, {"y", 0.0}, {"z", 1.5}, {"pitch", 0.0},
                              {"yaw", 0.0}, {"roll", 0.0}, {"enabled", true}, {"attributes", {{"image_size_x", 1600}}}}};
    cfgfile::ForSave(cfg, "localhost", 2000);
    const fs::path p = dir / "roundtrip.json";
    std::string err;
    const bool ok = cfgfile::WriteFileReplace(p.u8string(), cfg.dump(2), err);
    std::ifstream f(p);
    json back = cfgfile::FileOverDefaults(defaults, defaults, json::parse(f, nullptr, false));
    const int fixed = cfgfile::ConformConfig(back, defaults);
    Check(ok && fixed == 0 && back == cfg, "saved and loaded again: the same config, no warning", fixed);
  }

  // Writing a file: replaced whole, never half-written.
  {
    const fs::path p = dir / "cosim_config.json";
    std::string err;
    bool ok = cfgfile::WriteFileReplace(p.u8string(), "{\"a\": 1}", err) &&
              cfgfile::WriteFileReplace(p.u8string(), "{\"b\": 2}", err);
    Check(ok && Read(p) == "{\"b\": 2}" && NoTmpLeft(dir), "write, then replace: the new text, no .tmp left", err);
    err.clear();
    ok = cfgfile::WriteFileReplace((dir / "no_such_dir" / "x.json").u8string(), "{}", err);
    Check(!ok && !err.empty(), "a missing directory: false with a reason", err);
    const fs::path ro = dir / "readonly";
    fs::create_directory(ro);
    const fs::path q = ro / "cosim_config.json";
    { std::ofstream(q) << "old"; }
    fs::permissions(ro, fs::perms::owner_read | fs::perms::owner_exec);
    const bool writable = static_cast<bool>(std::ofstream(ro / "probe"));
    if (writable) {
      std::printf("SKIP  a read-only directory (this user can write there anyway)\n");
      std::error_code ec;
      fs::remove(ro / "probe", ec);
    } else {
      err.clear();
      ok = cfgfile::WriteFileReplace(q.u8string(), "new", err);
      Check(!ok && !err.empty() && Read(q) == "old" && NoTmpLeft(ro),
            "a read-only directory: false with a reason, the old file kept", err);
    }
    fs::permissions(ro, fs::perms::owner_all);
  }

  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
