// The run config as a file: what loading, saving and repairing do to it,
// without the GUI (cosim_gui/tests/config_file_test.cpp checks it).
#pragma once

#include <string>

#include "json.hpp"

using json = nlohmann::json;

namespace cfgfile {

// A loaded file over the backend's defaults, like settings.load_dict
// (run_cosim.py): not over the config loaded before, whose sections a partial
// file would keep. Only the vehicle and spawn point picked on the 车辆 page
// stay unless the file sets them. Without the defaults yet: over current.
json FileOverDefaults(const json& defaults, const json& current, const json& file);

// Makes cfg match the shape of defaults (the backend's default config): values
// of the wrong type (a hand-edited file) would make the pages throw. Numbers
// written as text become numbers. Returns how many values were changed.
int ConformConfig(json& cfg, const json& defaults);

// The GUI keeps the CarSim driver in drive.cosim_driver; the backend and
// run_cosim.py read run.driver: a run and a saved file always get the same.
void SyncRunDriver(json& cfg);

// What a saved file holds besides the pages: the driver in step, and the
// CARLA server this GUI talks to (run_cosim.py --config connects there).
void ForSave(json& cfg, const std::string& carla_host, int carla_port);

// Writes the file next to the target and renames it over the target, so a
// full disk or a crash never leaves a truncated file. false + err on failure.
bool WriteFileReplace(const std::string& path, const std::string& text, std::string& err);

}  // namespace cfgfile
