cmake_minimum_required(VERSION 3.16)
project(CarlaCoSimStudio LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# All third-party sources are vendored under third_party/, so the build works
# offline on both Windows (VS 2022) and Ubuntu.
set(TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
if(UNIX AND NOT APPLE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)  # X11 only: no wayland-scanner needed
endif()
add_subdirectory(${TP}/glfw ${CMAKE_BINARY_DIR}/glfw EXCLUDE_FROM_ALL)

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

add_library(imgui STATIC
  ${TP}/imgui/imgui.cpp
  ${TP}/imgui/imgui_draw.cpp
  ${TP}/imgui/imgui_tables.cpp
  ${TP}/imgui/imgui_widgets.cpp
  ${TP}/imgui/imgui_demo.cpp
  ${TP}/imgui/backends/imgui_impl_glfw.cpp
  ${TP}/imgui/backends/imgui_impl_opengl3.cpp
  ${TP}/implot/implot.cpp
  ${TP}/implot/implot_items.cpp)
target_include_directories(imgui PUBLIC ${TP}/imgui ${TP}/imgui/backends ${TP}/implot ${TP}/fonts)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

add_executable(carla_cosim_studio
  src/main.cpp
  src/app.cpp
  src/ui_kit.cpp
  src/ui_layout.cpp
  src/panels.cpp
  src/rig_editor.cpp
  src/backend_client.cpp
  src/platform.cpp)
target_compile_definitions(carla_cosim_studio PRIVATE COSIM_FONT_DIR="${TP}/fonts")
# Ship the icon font next to the executable so the build folder is portable.
add_custom_command(TARGET carla_cosim_studio POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:carla_cosim_studio>/fonts
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TP}/fonts/fa-solid-900.ttf $<TARGET_FILE_DIR:carla_cosim_studio>/fonts/)
target_include_directories(carla_cosim_studio PRIVATE src ${TP}/json)
target_link_libraries(carla_cosim_studio PRIVATE imgui Threads::Threads)
if(WIN32)
  target_link_libraries(carla_cosim_studio PRIVATE ws2_32)
  target_compile_definitions(carla_cosim_studio PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _WIN32_WINNT=0x0601)
  # GUI app without a console window, keeping main() as the entry point.
  set_target_properties(carla_cosim_studio PROPERTIES WIN32_EXECUTABLE ON)
  if(MSVC)
    # Sources are UTF-8 (Chinese UI strings); MSVC otherwise reads them as ANSI.
    target_compile_options(carla_cosim_studio PRIVATE /utf-8)
    target_compile_options(imgui PRIVATE /utf-8)
    target_link_options(carla_cosim_studio PRIVATE /ENTRY:mainCRTStartup)
  else()  # MinGW: GCC already reads UTF-8; -mwindows is the GUI subsystem
    target_link_options(carla_cosim_studio PRIVATE -mwindows)
  endif()
endif()
