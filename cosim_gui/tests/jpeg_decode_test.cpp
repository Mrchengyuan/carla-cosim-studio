// Checks src/jpeg_decode.cpp (stb_image, JPEG only) on a live view frame the
// backend encoded for a remote GUI: its size and, within JPEG's loss, its
// pixels; broken frames are refused. carsim_carla_bridge/tests/test_offline_remote_gui.py
// writes the frame and builds and runs this; by hand, in cosim_gui/:
//   c++ -std=c++17 -Isrc -Ithird_party/stb tests/jpeg_decode_test.cpp src/jpeg_decode.cpp -o /tmp/jpeg_decode_test
//   /tmp/jpeg_decode_test <frame.jpg> <frame.rgb (the pixels encoded)> <width> <height>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "jpeg_decode.h"

static int g_pass = 0, g_fail = 0;

static void Check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  ++(ok ? g_pass : g_fail);
}

static std::vector<unsigned char> Read(const char* path) {
  std::ifstream f(path, std::ios::binary);
  return std::vector<unsigned char>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

int main(int argc, char** argv) {
  if (argc < 5) {
    std::fprintf(stderr, "usage: jpeg_decode_test <frame.jpg> <frame.rgb> <width> <height>\n");
    return 2;
  }
  const std::vector<unsigned char> jpg = Read(argv[1]), raw = Read(argv[2]);
  const int want_w = std::atoi(argv[3]), want_h = std::atoi(argv[4]);
  std::vector<unsigned char> rgb;
  int w = 0, h = 0;
  Check(DecodeJpeg(jpg, rgb, w, h), "the backend's JPEG frame decodes");
  Check(w == want_w && h == want_h, "its size: " + std::to_string(w) + " x " + std::to_string(h));
  Check(rgb.size() == raw.size(), "3 bytes per pixel");
  double diff = 1e9;
  if (rgb.size() == raw.size() && !raw.empty()) {
    diff = 0;
    for (size_t i = 0; i < raw.size(); ++i) diff += std::abs(static_cast<int>(rgb[i]) - static_cast<int>(raw[i]));
    diff /= static_cast<double>(raw.size());
  }
  Check(diff < 4.0, "mean difference to the pixels encoded: " + std::to_string(diff) + " per channel (quality 80)");
  // A cut-off frame and an empty one: refused, the last frame kept.
  const std::vector<unsigned char> keep = rgb;
  const int keep_w = w, keep_h = h;
  const std::vector<unsigned char> cut(jpg.begin(), jpg.begin() + std::min<size_t>(jpg.size(), 20));
  Check(!DecodeJpeg(cut, rgb, w, h) && rgb == keep && w == keep_w && h == keep_h, "a cut-off frame is refused");
  Check(!DecodeJpeg(std::vector<unsigned char>(), rgb, w, h) && rgb == keep, "an empty frame is refused");
  // Only JPEG is compiled in (STBI_ONLY_JPEG): a PNG signature is not read.
  const std::vector<unsigned char> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n', 0, 0, 0, 13, 'I', 'H', 'D', 'R'};
  Check(!DecodeJpeg(png, rgb, w, h) && rgb == keep, "not a JPEG: refused");
  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
