// CARLA CoSim Studio entry point: GLFW window + OpenGL3 + Dear ImGui + ImPlot.
#include <csignal>
#include <cstdio>
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

// CJK text font + Font Awesome icons merged in; a bold variant for titles
// and a large one for KPI numbers.
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
#else
  const std::string regular = FirstExisting({override_path, "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                                             "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
                                             "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"});
  const std::string bold = FirstExisting({"/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                                          "/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc", regular});
#endif
  const float size = 16.0f * scale;
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
    // Titles only use common characters: a smaller range keeps the atlas small.
    f.bold = io.Fonts->AddFontFromFileTTF(bold.c_str(), size, &cfg, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
    MergeIcons(size, icons);
    f.big = io.Fonts->AddFontFromFileTTF(bold.c_str(), size * 1.55f, &cfg, io.Fonts->GetGlyphRangesDefault());
    std::printf("fonts: %s | %s | icons %s\n", regular.c_str(), bold.c_str(), icons.empty() ? "(missing)" : icons.c_str());
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
  ps.Colors[ImPlotCol_LegendBg] = ImVec4(p.card.x, p.card.y, p.card.z, 0.85f);
  ps.PlotPadding = ImVec2(6, 6);
  ps.PlotDefaultSize = ImVec2(400, 150);
}

int main(int argc, char** argv) {
  glfwSetErrorCallback(GlfwError);
  std::signal(SIGTERM, OnQuitSignal);
  std::signal(SIGINT, OnQuitSignal);
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#ifdef GLFW_SCALE_TO_MONITOR
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
#endif
  GLFWwindow* window = glfwCreateWindow(1680, 1000, "CARLA CoSim Studio", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  float xs = 1.0f, ys = 1.0f;
  glfwGetWindowContentScale(window, &xs, &ys);
  const float scale = xs > 0.5f ? xs : 1.0f;

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
