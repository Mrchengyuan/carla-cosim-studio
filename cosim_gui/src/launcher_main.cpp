// Remote launcher entry point (启动远程仿真.exe): GLFW window + OpenGL3 + Dear
// ImGui, the same look as the GUI. See launcher.h.
#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#include <windows.h>
#endif
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "launcher.h"
#include "platform.h"
#include "ui_fonts.h"
#include "ui_kit.h"

#include <GLFW/glfw3.h>

static volatile std::sig_atomic_t g_quit_signal = 0;
static void OnQuitSignal(int) { g_quit_signal = 1; }

static void GlfwError(int code, const char* desc) { std::fprintf(stderr, "GLFW %d: %s\n", code, desc); }

#ifdef _WIN32
// A second double-click brings the running launcher to the front instead of
// starting another one (its tunnel would find the ports taken).
static BOOL CALLBACK ShowOther(HWND hwnd, LPARAM) {
  wchar_t cls[64], title[128];
  GetClassNameW(hwnd, cls, 64);
  GetWindowTextW(hwnd, title, 128);
  if (std::wstring(cls) == L"GLFW30" && std::wstring(title).rfind(L"远程仿真", 0) == 0) {
    ShowWindow(hwnd, IsIconic(hwnd) ? SW_RESTORE : SW_SHOW);
    SetForegroundWindow(hwnd);
    return FALSE;
  }
  return TRUE;
}
#endif

int main(int argc_raw, char** argv_raw) {
  const std::vector<std::string> args = plat::Utf8Args(argc_raw, argv_raw);
  bool tour = false;
  for (const auto& a : args) tour = tour || a == "--tour";
#ifdef _WIN32
  HANDLE single = CreateMutexW(nullptr, TRUE, L"Local\\CarlaCoSimRemoteLauncher");
  if (single && GetLastError() == ERROR_ALREADY_EXISTS && !tour) {
    EnumWindows(ShowOther, 0);
    return 0;
  }
#endif
  glfwSetErrorCallback(GlfwError);
  std::signal(SIGTERM, OnQuitSignal);
  std::signal(SIGINT, OnQuitSignal);
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#ifdef GLFW_SCALE_TO_MONITOR
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
#endif
  int win_w = 1100, win_h = 780;
  bool size_given = false;
  float scale_override = 0.0f;
  for (size_t i = 1; i + 1 < args.size(); ++i) {
    if (args[i] == "--size") size_given = std::sscanf(args[i + 1].c_str(), "%dx%d", &win_w, &win_h) == 2;
    if (args[i] == "--scale") scale_override = static_cast<float>(std::atof(args[i + 1].c_str()));
    if (args[i] == "--test-pick") plat::SetTestPick(args[i + 1]);
  }
  GLFWwindow* window = glfwCreateWindow(win_w, win_h, "远程仿真启动器", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  if (!size_given) {  // never larger than the screen (1366x768 laptops, 150 % scaling)
    int wx = 0, wy = 0, ww = 0, wh = 0, fw = 0, fh = 0, l = 0, t = 0, r = 0, b = 0;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor()) glfwGetMonitorWorkarea(mon, &wx, &wy, &ww, &wh);
    glfwGetWindowSize(window, &fw, &fh);
    glfwGetWindowFrameSize(window, &l, &t, &r, &b);
    if (ww > 0 && wh > 0) {
      const int w = std::min(fw, ww - l - r), h = std::min(fh, wh - t - b);
      glfwSetWindowSize(window, w, h);
      glfwSetWindowPos(window, wx + (ww - w) / 2, wy + std::max(t, (wh - h) / 2));
    }
  }
  float xs = 1.0f, ys = 1.0f;
  glfwGetWindowContentScale(window, &xs, &ys);
  const float scale = scale_override > 0.3f ? scale_override : (xs > 0.5f ? xs : 1.0f);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;
  auto launcher = std::make_unique<Launcher>();
  launcher->Init(args);
  ui::ApplyTheme(launcher->Dark(), scale);
  ui::LoadFonts(scale, "");
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  std::string shown_title;
  while (!launcher->WantsQuit()) {
    if (g_quit_signal) break;
    // Idle most of the time (the laptop is weak and runs CarSim): a few frames a
    // second, 30 while something moves; input wakes it at once.
    if (launcher->Animating()) glfwWaitEventsTimeout(1.0 / 30.0);
    else glfwWaitEventsTimeout(0.25);
    if (glfwWindowShouldClose(window)) {
      glfwSetWindowShouldClose(window, GLFW_FALSE);
      launcher->AskQuit();
      glfwRestoreWindow(window);
    }
    switch (launcher->TakeWindowRequest()) {
      case 1: glfwIconifyWindow(window); break;
      case 2:
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) glfwRestoreWindow(window);
        glfwFocusWindow(window);
        break;
      default: break;
    }
    if (launcher->ThemeChanged()) ui::ApplyTheme(launcher->Dark(), scale);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    launcher->BeforeNewFrame();
    ImGui::NewFrame();
    launcher->Frame();
    ImGui::Render();
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    glViewport(0, 0, w, h);
    const ImVec4 bg = ui::Colors().bg;
    glClearColor(bg.x, bg.y, bg.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    launcher->AfterRender(w, h);
    glfwSwapBuffers(window);
    const std::string title = launcher->WindowTitle();
    if (title != shown_title) glfwSetWindowTitle(window, (shown_title = title).c_str());
  }
  glfwHideWindow(window);
  launcher.reset();  // stops the CarSim service, the connection and the GUI in order
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
