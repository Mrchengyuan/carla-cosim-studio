#include "ui_fonts.h"

#include <cstdio>
#include <string>
#include <vector>

#include "IconsFontAwesome6.h"
#include "imgui.h"
#include "platform.h"
#include "ui_kit.h"
#include "ui_glyphs.inc"

namespace ui {

namespace {

std::string FirstExisting(const std::vector<std::string>& paths) {
  for (const auto& p : paths)
    if (!p.empty() && plat::FileExists(p)) return p;
  return "";
}

void MergeIcons(float size, const std::string& icon_path) {
  if (icon_path.empty()) return;
  static const ImWchar kIconRange[] = {ICON_MIN_FA, ICON_MAX_16_FA, 0};
  ImFontConfig cfg;
  cfg.MergeMode = true;
  cfg.PixelSnapH = true;
  cfg.GlyphMinAdvanceX = size;  // monospaced icons line up in lists
  cfg.OversampleH = cfg.OversampleV = 1;
  ImGui::GetIO().Fonts->AddFontFromFileTTF(icon_path.c_str(), size * 0.9f, &cfg, kIconRange);
}

}  // namespace

void LoadFonts(float scale, const std::string& override_path) {
  ImGuiIO& io = ImGui::GetIO();
  const std::string exe = plat::ExecutableDir();
  const std::string icons = FirstExisting({exe + "/fonts/fa-solid-900.ttf",
#ifdef COSIM_FONT_DIR
                                           std::string(COSIM_FONT_DIR) + "/fa-solid-900.ttf",
#endif
                                           exe + "/../third_party/fonts/fa-solid-900.ttf"});
#ifdef _WIN32
  const std::string regular = FirstExisting({override_path, "C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/msyh.ttf",
                                             "C:/Windows/Fonts/simhei.ttf", "C:/Windows/Fonts/simsun.ttc"});
  const std::string bold = FirstExisting({"C:/Windows/Fonts/msyhbd.ttc", "C:/Windows/Fonts/msyhbd.ttf", regular});
  const std::string mono = FirstExisting({"C:/Windows/Fonts/consola.ttf", "C:/Windows/Fonts/cour.ttf"});
#else
  const std::string regular = FirstExisting({override_path, "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                                             "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
                                             "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"});
  const std::string bold = FirstExisting({"/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                                          "/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc", regular});
  const std::string mono = FirstExisting({"/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
                                          "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
                                          "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf"});
#endif
  const float size = 15.0f * scale;
  ui::Fonts& f = ui::GetFonts();
  // CJK plus the symbols the UI uses (→ ↑ ↓ ← × ≈ ● ° …), which the stock
  // Chinese range leaves out.
  static ImVector<ImWchar> ranges;
  if (ranges.empty()) {
    ImFontGlyphRangesBuilder b;
    b.AddRanges(io.Fonts->GetGlyphRangesChineseFull());
    static const ImWchar kExtra[] = {0x2000, 0x206F, 0x2190, 0x21FF, 0x2200, 0x22FF, 0x2460, 0x24FF,
                                     0x25A0, 0x25FF, 0x2600, 0x26FF, 0};
    b.AddRanges(kExtra);
    b.BuildRanges(&ranges);
  }
  ImFontConfig cfg;
  cfg.OversampleH = cfg.OversampleV = 1;  // keeps the ~20k-glyph atlas small
  if (!regular.empty()) {
    f.regular = io.Fonts->AddFontFromFileTTF(regular.c_str(), size, &cfg, ranges.Data);
    MergeIcons(size, icons);
    // Bold / title fonts: common Chinese plus every character the UI source
    // uses (ui_glyphs.inc), instead of the full range, to keep the atlas small.
    static ImVector<ImWchar> ui_ranges;
    if (ui_ranges.empty()) {
      ImFontGlyphRangesBuilder b;
      b.AddRanges(io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
      b.AddText(kUiGlyphs);
      b.BuildRanges(&ui_ranges);
    }
    f.bold = io.Fonts->AddFontFromFileTTF(bold.c_str(), size, &cfg, ui_ranges.Data);
    MergeIcons(size, icons);
    f.title = io.Fonts->AddFontFromFileTTF(bold.c_str(), size * 1.3f, &cfg, ui_ranges.Data);
    MergeIcons(size * 1.3f, icons);
    // Readouts only show digits and a few symbols.
    const std::string digits = mono.empty() ? bold : mono;
    f.mono = io.Fonts->AddFontFromFileTTF(digits.c_str(), size * 0.93f, &cfg, io.Fonts->GetGlyphRangesDefault());
    f.mono_big = io.Fonts->AddFontFromFileTTF(digits.c_str(), size * 1.6f, &cfg, io.Fonts->GetGlyphRangesDefault());
    std::printf("fonts: %s | %s | mono %s | icons %s\n", regular.c_str(), bold.c_str(), digits.c_str(),
                icons.empty() ? "(missing)" : icons.c_str());
  } else {
    std::fprintf(stderr, "warning: no CJK font found, Chinese text will not render\n");
    ImFontConfig d;
    d.SizePixels = size;
    f.regular = io.Fonts->AddFontDefault(&d);
    MergeIcons(size, icons);
  }
  io.FontDefault = f.regular;
}

}  // namespace ui
