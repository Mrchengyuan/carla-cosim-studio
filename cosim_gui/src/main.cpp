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
#include "ui_fonts.h"
#include "ui_kit.h"

#include <GLFW/glfw3.h>

// SIGTERM / SIGINT (logout, kill, Ctrl+C) end the main loop like closing the
// window, so the app still asks the backend to remove its CARLA actors.
static volatile std::sig_atomic_t g_quit_signal = 0;
static void OnQuitSignal(int) { g_quit_signal = 1; }

static void GlfwError(int code, const char* desc) { std::fprintf(stderr, "GLFW %d: %s\n", code, desc); }

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
  ui::LoadFonts(scale, app.FontPathOverride());

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  std::string shown_title = "CARLA CoSim Studio";
  while (!app.WantsQuit() && !g_quit_signal) {
    // Minimised: swapping does not wait for a hidden window on many drivers, and
    // the loop would spin a core CarSim or CARLA need; still take in the
    // backend's messages about 60 times a second.
    if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) glfwWaitEventsTimeout(0.016);
    else glfwPollEvents();
    // Closing the window asks first when the config has unsaved changes.
    if (glfwWindowShouldClose(window)) {
      glfwSetWindowShouldClose(window, GLFW_FALSE);
      app.AskQuit();
    }
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
    const std::string title = app.WindowTitle();
    if (title != shown_title) glfwSetWindowTitle(window, (shown_title = title).c_str());
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
