@echo off
rem One-click start: CARLA (if not running) + CARLA CoSim Studio.
rem   start_studio.bat          original CARLA 0.9.16
rem   start_studio.bat mod      modified CARLA (external-dynamics API)
rem Closing the GUI also stops the CARLA server this script started.
chcp 65001 >nul
setlocal
call "%~dp0env.bat"

rem (Paths are quoted inside ( ) blocks: a ")" in them, e.g. "Program Files (x86)",
rem would end the block.)
if /i "%~1"=="mod" (
  set "CROOT=%CARLA_MOD_ROOT%"
  set "PORT=%CARLA_MOD_PORT%"
) else (
  set "CROOT=%CARLA_ROOT%"
  set "PORT=%CARLA_PORT%"
)
set "EXE=%CROOT%\CarlaUE4.exe"
if not exist "%STUDIO_EXE%" (
  echo [错误] 找不到界面程序 "%STUDIO_EXE%"
  echo        请把 Release 里的 CARLA_CoSim_Studio_Windows.zip 解压到 "%COSIM_ROOT%"
  pause & exit /b 1
)

set STARTED=0
call :listening %PORT%
if errorlevel 1 (
  if not exist "%EXE%" (
    echo [错误] 找不到 CARLA："%EXE%"
    echo        请检查 scripts\windows\env.bat 里的 CARLA_ROOT / CARLA_MOD_ROOT
    pause & exit /b 1
  )
  echo 正在启动 CARLA（端口 %PORT%）...
  start "CARLA" /min "%EXE%" -RenderOffScreen -nosound -carla-rpc-port=%PORT%
  set STARTED=1
  call :wait_port %PORT% 300
  if errorlevel 1 (
    echo [错误] CARLA 没有启动成功（中途退出，或 300 秒内没有打开端口）
    call "%~dp0stop_carla.bat" quiet "%CROOT%"
    pause & exit /b 1
  )
)

echo 打开 CARLA CoSim Studio ...
"%STUDIO_EXE%" --python "%COSIM_PYTHON%" --backend-dir "%COSIM_ROOT%\carsim_carla_bridge" --carla-port %PORT% --auto-connect

rem Only the CARLA this script started (another one, e.g. the other kind, keeps running).
if "%STARTED%"=="1" call "%~dp0stop_carla.bat" quiet "%CROOT%"
endlocal
exit /b 0

:listening
rem errorlevel 0 when something listens on 127.0.0.1:%1. Matched by the empty
rem foreign address, not the word LISTENING: netstat translates it.
netstat -an | findstr /r /c:"127.0.0.1:%1  *0.0.0.0:0 " /c:"0.0.0.0:%1  *0.0.0.0:0 " >nul
exit /b %errorlevel%

:wait_port
set /a "_left=%2/2"
:wait_loop
call :listening %1
if not errorlevel 1 (timeout /t 3 /nobreak >nul & exit /b 0)
rem CARLA gone (crashed while loading): no point waiting any longer.
tasklist /fi "imagename eq CarlaUE4-Win64-Shipping.exe" | find /i "CarlaUE4" >nul && goto wait_alive
tasklist /fi "imagename eq CarlaUE4.exe" | find /i "CarlaUE4" >nul && goto wait_alive
exit /b 1
:wait_alive
set /a "_left-=1"
if %_left% LEQ 0 exit /b 1
timeout /t 2 /nobreak >nul
goto wait_loop
