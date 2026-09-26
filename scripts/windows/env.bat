@echo off
rem ===================================================================
rem  CARLA CoSim Studio - Windows settings (edit here, or set the same
rem  variables before calling the scripts). Paths follow the layout in
rem  docs\Windows使用指南.md
rem ===================================================================
set "COSIM_ROOT=%~dp0..\.."
for %%I in ("%COSIM_ROOT%") do set "COSIM_ROOT=%%~fI"

rem Original CARLA 0.9.16 (unzipped). Both CARLA_0.9.16\CarlaUE4.exe and
rem CARLA_0.9.16\WindowsNoEditor\CarlaUE4.exe layouts are accepted.
if not defined CARLA_ROOT set "CARLA_ROOT=%COSIM_ROOT%\CARLA_0.9.16"
if exist "%CARLA_ROOT%\WindowsNoEditor\CarlaUE4.exe" set "CARLA_ROOT=%CARLA_ROOT%\WindowsNoEditor"

rem Modified CARLA: folder that contains CarlaUE4.exe from "make package"
rem (e.g. C:\carla\Build\UE4Carla\0.9.16\WindowsNoEditor). "make package" names
rem the version folder after git describe (e.g. 0.9.16-dirty or a commit id):
rem when the default is not there, the first build found under C:\carla is used.
if defined CARLA_MOD_ROOT goto mod_root_done
set "CARLA_MOD_ROOT=C:\carla\Build\UE4Carla\0.9.16\WindowsNoEditor"
if exist "C:\carla\Build\UE4Carla\0.9.16\WindowsNoEditor\CarlaUE4.exe" goto mod_root_done
for /d %%D in ("C:\carla\Build\UE4Carla\*") do if exist "%%~D\WindowsNoEditor\CarlaUE4.exe" set "CARLA_MOD_ROOT=%%~D\WindowsNoEditor"
:mod_root_done

rem GUI: the unzipped release, or your own build.
if not defined STUDIO_EXE set "STUDIO_EXE=%COSIM_ROOT%\CARLA_CoSim_Studio_Windows\carla_cosim_studio.exe"
if not exist "%STUDIO_EXE%" if exist "%COSIM_ROOT%\cosim_gui\build\Release\carla_cosim_studio.exe" set "STUDIO_EXE=%COSIM_ROOT%\cosim_gui\build\Release\carla_cosim_studio.exe"

rem Python that runs the backend (needs the carla package).
if not defined COSIM_PYTHON set "COSIM_PYTHON=%COSIM_ROOT%\venv\Scripts\python.exe"
if not exist "%COSIM_PYTHON%" set "COSIM_PYTHON=python"

rem CARLA's "agents" package (route planner) is not in the pip wheel.
if not defined CARLA_PYTHONAPI set "CARLA_PYTHONAPI=%CARLA_ROOT%\PythonAPI\carla"

if not defined CARLA_PORT set "CARLA_PORT=2000"
if not defined CARLA_MOD_PORT set "CARLA_MOD_PORT=3000"
