@echo off
rem Stop every CARLA server (original and modified) and free the GPU.
chcp 65001 >nul
rem stop_carla.bat [quiet] [folder]: with a folder, only the CARLA started from it
rem (start_studio.bat passes the one it started); without, every CARLA.
if "%~2"=="" goto stop_all
set "STOP_ROOT=%~2"
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.ExecutablePath -and $_.ExecutablePath.StartsWith($env:STOP_ROOT, [StringComparison]::OrdinalIgnoreCase) } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
goto stopped
:stop_all
taskkill /IM CarlaUE4-Win64-Shipping.exe /F >nul 2>&1
taskkill /IM CarlaUE4.exe /F >nul 2>&1
:stopped
if /i not "%~1"=="quiet" (echo CARLA 已关闭 & timeout /t 2 >nul)
