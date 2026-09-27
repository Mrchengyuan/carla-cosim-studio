#include "config_file.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace fs = std::filesystem;

namespace {
// "1600" -> 1600, "0.5" -> 0.5: numbers written as text (hand-edited files,
// CARLA-style sensor attributes). false when v is no such string.
bool NumberFromString(const json& v, json& out) {
  if (!v.is_string() || v.get_ref<const std::string&>().empty()) return false;
  const std::string& s = v.get_ref<const std::string&>();
  char* end = nullptr;
  const double d = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size() || !std::isfinite(d)) return false;
  if (s.find_first_of(".eE") == std::string::npos && std::fabs(d) < 9e15) out = static_cast<long long>(d);
  else out = d;
  return true;
}

// 1 / 0, "true" / "false": what a hand-edited "enabled" may hold. false when v is none of these.
bool BoolFrom(const json& v, bool& out) {
  if (v.is_number()) {
    out = v.get<double>() != 0.0;
    return true;
  }
  if (!v.is_string()) return false;
  std::string s = v.get<std::string>();
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (s != "true" && s != "false" && s != "1" && s != "0") return false;
  out = s == "true" || s == "1";
  return true;
}

// Sensor attributes the pages read as numbers (rig editor, estimate, previews).
const char* const kSensorNumberAttrs[] = {"image_size_x", "image_size_y", "fov", "max_distance", "channels", "range",
                                          "points_per_second", "rotation_frequency", "upper_fov", "lower_fov",
                                          "horizontal_fov", "vertical_fov"};

// Makes v match the shape of def (the backend's default config): values of the
// wrong type (a hand-edited file) would make the pages throw. Numbers written
// as text become numbers. Returns how many values were changed.
int Conform(json& v, const json& def) {
  int fixed = 0;
  if (def.is_object()) {
    if (!v.is_object()) { v = def; return 1; }
    for (auto& kv : def.items()) {
      if (!v.contains(kv.key())) v[kv.key()] = kv.value();
      else fixed += Conform(v[kv.key()], kv.value());
    }
  } else if (def.is_number()) {
    json n;
    if (NumberFromString(v, n)) { v = n; return 1; }
    if (!v.is_number()) { v = def; return 1; }
  } else if (def.is_string()) {
    if (!v.is_string()) { v = def; return 1; }
  } else if (def.is_boolean()) {
    bool b = false;
    if (!v.is_boolean()) { v = BoolFrom(v, b) ? json(b) : def; return 1; }
  } else if (def.is_array()) {
    if (!v.is_array()) { v = def; return 1; }
    if (!def.empty()) {  // element type from the first default element
      json keep = json::array();
      for (auto& e : v) {
        if ((def[0].is_string() && e.is_string()) || (def[0].is_number() && e.is_number()) || def[0].is_object()) {
          if (def[0].is_object()) fixed += Conform(e, def[0]);
          keep.push_back(e);
        } else {
          ++fixed;
        }
      }
      v = keep;
    }
  }
  return fixed;
}
}  // namespace

namespace cfgfile {

json FileOverDefaults(const json& defaults, const json& current, const json& file) {
  json n = defaults.is_object() ? defaults : current;
  if (defaults.is_object() && current.contains("carla") && current["carla"].is_object())
    for (const char* k : {"vehicle", "spawn_index"})
      if (current["carla"].contains(k)) n["carla"][k] = current["carla"][k];
  n.merge_patch(file);
  return n;
}

int ConformConfig(json& cfg, const json& defaults) {
  if (!defaults.is_object()) return 0;
  // The reference point is "front_axle" or [x, y, z], the CARLA interface
  // "auto" or true / false: the default's type (a string) must not replace a
  // point set or an interface forced on the CarSim page.
  json ref, ext;
  if (cfg.contains("sync") && cfg["sync"].is_object() && cfg["sync"].contains("reference_point")) {
    const json& r = cfg["sync"]["reference_point"];
    if (r.is_array() && r.size() == 3 && std::all_of(r.begin(), r.end(), [](const json& v) { return v.is_number(); })) ref = r;
  }
  if (cfg.contains("sync") && cfg["sync"].is_object() && cfg["sync"].value("use_external_api", json()).is_boolean())
    ext = cfg["sync"]["use_external_api"];
  // Out of Conform's way (it would count them as wrong types), back after it.
  if (!ref.is_null()) cfg["sync"].erase("reference_point");
  if (!ext.is_null()) cfg["sync"].erase("use_external_api");
  int fixed = Conform(cfg, defaults);
  if (!ref.is_null()) cfg["sync"]["reference_point"] = ref;
  if (!ext.is_null()) cfg["sync"]["use_external_api"] = ext;
  // Never read by anything (older configs have them): not kept, so that a
  // saved file does not suggest they set the map or the weather.
  for (const char* k : {"map", "weather"}) cfg["carla"].erase(std::string(k));
  // The GUI keeps the CarSim driver in drive.cosim_driver (not in the
  // defaults); a file without it (hand-written, settings.save_dict) runs run.driver.
  json& dr = cfg["drive"];
  if (dr.contains("cosim_driver") && !dr["cosim_driver"].is_string()) { dr.erase("cosim_driver"); ++fixed; }
  if (!dr.contains("cosim_driver")) dr["cosim_driver"] = cfg["run"]["driver"];
  // Rig sensors: objects with numbers where numbers belong; numbers written
  // as text (CARLA attributes often are) become numbers.
  json& sensors = cfg["rig"]["sensors"];
  json keep = json::array();
  for (auto& s : sensors) {
    if (!s.is_object()) { ++fixed; continue; }
    for (const char* k : {"x", "y", "z", "pitch", "yaw", "roll"})
      if (s.contains(k) && !s[k].is_number()) { json n; s[k] = NumberFromString(s[k], n) ? n : json(0.0); ++fixed; }
    for (const char* k : {"name", "type"})
      if (!s.value(k, json()).is_string()) { s[k] = std::string(k) == "type" ? "rgb" : "sensor"; ++fixed; }
    bool en = true;
    if (s.contains("enabled") && !s["enabled"].is_boolean()) { s["enabled"] = BoolFrom(s["enabled"], en) ? en : true; ++fixed; }
    if (!s.value("attributes", json()).is_object()) { s["attributes"] = json::object(); ++fixed; }
    json& a = s["attributes"];
    for (auto& kv : a.items()) {
      json n;
      if (NumberFromString(kv.value(), n)) kv.value() = n;
    }
    for (const char* k : kSensorNumberAttrs)
      if (a.contains(k) && !a[k].is_number()) { a.erase(std::string(k)); ++fixed; }  // the default is used
    keep.push_back(s);
  }
  sensors = keep;
  return fixed;
}

void SyncRunDriver(json& cfg) {
  if (cfg.contains("drive") && cfg["drive"].contains("cosim_driver"))
    cfg["run"]["driver"] = cfg["drive"]["cosim_driver"];
}

void ForSave(json& cfg, const std::string& carla_host, int carla_port) {
  SyncRunDriver(cfg);
  cfg["carla"]["host"] = carla_host;
  cfg["carla"]["port"] = carla_port;
}

bool WriteFileReplace(const std::string& path, const std::string& text, std::string& err) {
  const fs::path target = fs::u8path(path), tmp = fs::u8path(path + ".tmp");
  std::error_code ec;
  {
    std::ofstream f(tmp);
    if (f) {
      f << text;
      f.close();
    }
    if (!f) {
      fs::remove(tmp, ec);
      err = "写入失败（目录不存在、没有写权限或磁盘已满）";
      return false;
    }
  }
  fs::rename(tmp, target, ec);
#ifdef _WIN32
  if (ec) {  // some runtimes do not replace an existing file
    std::error_code ec2;
    fs::remove(target, ec2);
    fs::rename(tmp, target, ec);
  }
#endif
  if (ec) {
    err = ec.message();
    fs::remove(tmp, ec);
    return false;
  }
  return true;
}

}  // namespace cfgfile
