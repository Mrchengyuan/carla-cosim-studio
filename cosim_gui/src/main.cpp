// CARLA CoSim Studio entry point: GLFW window + OpenGL3 + Dear ImGui + ImPlot.
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "app.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "platform.h"
#include "ui_kit.h"
#include "ui_glyphs.inc"

#include <GLFW/glfw3.h>

// SIGTERM / SIGINT (logout, kill, Ctrl+C) end the main loop like closing the
// window, so the app still asks the backend to remove its CARLA actors.
static volatile std::sig_atomic_t g_quit_signal = 0;
static void OnQuitSignal(int) { g_quit_signal = 1; }

static void GlfwError(int code, const char* desc) { std::fprintf(stderr, "GLFW %d: %s\n", code, desc); }

static std::string FirstExisting(const std::vector<std::string>& paths) {
  for (const auto& p : paths)
    if (!p.empty() && plat::FileExists(p)) return p;
  return "";
}

static void MergeIcons(float size, const std::string& icon_path) {
  if (icon_path.empty()) return;
  static const ImWchar kIconRange[] = {ICON_MIN_FA, ICON_MAX_16_FA, 0};
  ImFontConfig cfg;
  cfg.MergeMode = true;
  cfg.PixelSnapH = true;
  cfg.GlyphMinAdvanceX = size;  // monospaced icons line up in lists
  cfg.OversampleH = cfg.OversampleV = 1;
  ImGui::GetIO().Fonts->AddFontFromFileTTF(icon_path.c_str(), size * 0.9f, &cfg, kIconRange);
}

// CJK text font + Font Awesome icons merged in; bold variants for section
// and page titles; a monospace font for instrument readouts.
static void LoadFonts(float scale, const std::string& override_path) {
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

static void ApplyAllThemes(bool dark, float scale) {
  ui::ApplyTheme(dark, scale);
  ImPlotStyle& ps = ImPlot::GetStyle();
  if (dark) ImPlot::StyleColorsDark(); else ImPlot::StyleColorsLight();
  const ui::Palette& p = ui::Colors();
  ps.Colors[ImPlotCol_PlotBg] = p.plot_bg;
  ps.Colors[ImPlotCol_FrameBg] = ImVec4(0, 0, 0, 0);
  ps.Colors[ImPlotCol_PlotBorder] = p.card_border;
  ps.Colors[ImPlotCol_LegendBg] = ImVec4(p.panel.x, p.panel.y, p.panel.z, 0.9f);
  ps.Colors[ImPlotCol_LegendBorder] = p.card_border;
  ps.Colors[ImPlotCol_AxisGrid] = ui::WithAlpha(p.text_dim, dark ? 0.16f : 0.22f);
  ps.Colors[ImPlotCol_AxisText] = p.text_dim;
  ps.Colors[ImPlotCol_TitleText] = p.text_dim;
  // Channel colours used by every chart: blue, orange, green, violet.
  static int colormap = -1;
  if (colormap < 0) {
    static const ImVec4 kSeries[] = {ImVec4(0.29f, 0.56f, 0.96f, 1), ImVec4(0.93f, 0.55f, 0.24f, 1),
                                     ImVec4(0.39f, 0.72f, 0.42f, 1), ImVec4(0.66f, 0.49f, 0.90f, 1)};
    colormap = ImPlot::AddColormap("Studio", kSeries, 4, false);
  }
  ps.Colormap = colormap;
  ps.PlotPadding = ImVec2(6, 6);
  ps.LegendPadding = ImVec2(6, 4);
  ps.PlotDefaultSize = ImVec2(400, 150);
}

int main(int argc_raw, char** argv_raw) {
  // UTF-8 arguments on every platform (Windows passes the ANSI code page).
  std::vector<std::string> args = plat::Utf8Args(argc_raw, argv_raw);
  std::vector<char*> argv_utf8;
  for (auto& a : args) argv_utf8.push_back(&a[0]);
  argv_utf8.push_back(nullptr);
  const int argc = static_cast<int>(args.size());
  char** argv = argv_utf8.data();
  glfwSetErrorCallback(GlfwError);
  std::signal(SIGTERM, OnQuitSignal);
  std::signal(SIGINT, OnQuitSignal);
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#ifdef GLFW_SCALE_TO_MONITOR
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
#endif
  // --size WxH / --scale S: window size and UI scale (e.g. for high-resolution screenshots).
  int win_w = 1680, win_h = 1000;
  bool size_given = false;
  float scale_override = 0.0f;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--size") size_given = std::sscanf(argv[i + 1], "%dx%d", &win_w, &win_h) == 2;
    if (std::string(argv[i]) == "--scale") scale_override = static_cast<float>(std::atof(argv[i + 1]));
  }
  GLFWwindow* window = glfwCreateWindow(win_w, win_h, "CARLA CoSim Studio", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  if (!size_given) {
    // Never larger than the screen: the default size (scaled up on high-DPI
    // screens) ran off a 1366x768 laptop, or a 1920x1080 one at 150 %.
    int wx = 0, wy = 0, ww = 0, wh = 0, fw = 0, fh = 0, l = 0, t = 0, r = 0, b = 0;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor()) glfwGetMonitorWorkarea(mon, &wx, &wy, &ww, &wh);
    glfwGetWindowSize(window, &fw, &fh);
    glfwGetWindowFrameSize(window, &l, &t, &r, &b);
    if (ww > 0 && wh > 0 && (fw + l + r > ww || fh + t + b > wh)) {
      glfwSetWindowSize(window, std::min(fw, ww - l - r), std::min(fh, wh - t - b));
      glfwSetWindowPos(window, wx + l, wy + t);
    }
  }

  float xs = 1.0f, ys = 1.0f;
  glfwGetWindowContentScale(window, &xs, &ys);
  const float scale = scale_override > 0.3f ? scale_override : (xs > 0.5f ? xs : 1.0f);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;

  auto app_ptr = std::make_unique<App>();
  App& app = *app_ptr;
  app.Init(argc, argv);
  ApplyAllThemes(app.DarkTheme(), scale);
  LoadFonts(scale, app.FontPathOverride());

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  while (!glfwWindowShouldClose(window) && !app.WantsQuit() && !g_quit_signal) {
    glfwPollEvents();
    if (app.ThemeChanged()) ApplyAllThemes(app.DarkTheme(), scale);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    app.BeforeNewFrame();
    ImGui::NewFrame();
    app.Frame();
    ImGui::Render();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    const ImVec4 bg = ui::Colors().bg;
    glClearColor(bg.x, bg.y, bg.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    app.AfterRender(w, h);
    glfwSwapBuffers(window);
  }

  // Destroy the app while GLFW is alive: it asks the backend to clean CARLA up.
  app_ptr.reset();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
