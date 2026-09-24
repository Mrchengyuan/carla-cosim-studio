@echo off
rem Stop every CARLA server (original and modified) and free the GPU.
chcp 65001 >nul
taskkill /IM CarlaUE4-Win64-Shipping.exe /F >nul 2>&1
taskkill /IM CarlaUE4.exe /F >nul 2>&1
if /i not "%~1"=="quiet" (echo CARLA 已关闭 & timeout /t 2 >nul)
