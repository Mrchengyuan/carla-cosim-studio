@echo off
chcp 65001 >nul
rem CARLA CoSim Studio, remote mode: this Windows laptop runs CarSim and the
rem GUI; CARLA, the backend and the control algorithm run on the cloud server
rem (docs\远程使用指南.md). Double-click it:
rem   1. window "云端连接": one SSH connection to the server. The server starts
rem      CARLA and the backend; ports 57120 / 57121 come here through it.
rem   2. window "CarSim 服务": the CarSim solver on this laptop
rem   3. the GUI. Closing it also closes the two windows above.
rem The two windows run this script again (:ssh_window / :service_window) with
rem the settings in the environment. No ( ) blocks: a ")" in a path, e.g.
rem "Program Files (x86)", would end one.
if "%~1"==":ssh_window" goto ssh_window
if "%~1"==":service_window" goto service_window
setlocal
title 启动远程仿真

rem ---- settings -------------------------------------------------------------
set "REMOTE_HOST=i.easy-ai.cloud"
set "REMOTE_PORT=32122"
set "REMOTE_USER=easyai"
rem Python for the CarSim service: empty = "py -3", else "python". To use a
rem particular one (e.g. a venv), put the full path of its python.exe here.
set "CARSIM_PYTHON="
rem ----------------------------------------------------------------------------

set "HERE=%~dp0"
set "SELF=%~f0"
set "KEY=%HERE%ssh\remote_key"
set "KNOWN=%HERE%ssh\known_hosts"
set "EXE=%HERE%carla_cosim_studio.exe"
set "SERVICE=%HERE%service\carsim_service.py"
set "STATE=%LOCALAPPDATA%\carla_cosim_studio"
if not exist "%STATE%" md "%STATE%"
set "LOG=%STATE%\remote_launch.log"
rem Written by the "云端连接" window when ssh has ended.
set "SSH_EXIT=%STATE%\remote_ssh_exit.txt"
set "SSH_PID="
set "SVC_PID="
> "%LOG%" echo %date% %time% 启动远程仿真.bat "%HERE%"

if not exist "%KEY%" goto not_unzipped
if not exist "%KNOWN%" goto not_unzipped
if not exist "%EXE%" goto not_unzipped
if not exist "%SERVICE%" goto not_unzipped

rem Windows' own OpenSSH client (Windows 10 1809 and later).
set "SSH=%SystemRoot%\System32\OpenSSH\ssh.exe"
if not exist "%SSH%" set "SSH=%SystemRoot%\Sysnative\OpenSSH\ssh.exe"
if not exist "%SSH%" for %%S in (ssh.exe) do set "SSH=%%~$PATH:S"
if not exist "%SSH%" goto no_ssh

rem OpenSSH refuses a key that others can read, and an unzipped file inherits
rem the folder's permissions: only this user may read it (by SID: user names
rem can be Chinese). Done every time: the folder may have been copied.
set "_sid="
for /f "tokens=2 delims=," %%S in ('whoami /user /fo csv /nh 2^>nul') do set "_sid=%%~S"
set "_who=%USERNAME%"
if defined _sid set "_who=*%_sid%"
icacls "%KEY%" /inheritance:r /grant:r "%_who%:R" >nul 2>&1
if errorlevel 1 call :say [警告] 没能设置钥匙文件 ssh\remote_key 的权限；“云端连接”窗口若提示 bad permissions，见使用指南的常见问题

rem Python with numpy (the CarSim service needs it).
set "PY="
if defined CARSIM_PYTHON set PY="%CARSIM_PYTHON%"
if not defined PY py -3 -c "import sys" >nul 2>&1 && set "PY=py -3"
if not defined PY python -c "import sys" >nul 2>&1 && set "PY=python"
if not defined PY goto no_python
%PY% -c "import numpy" >nul 2>&1
if errorlevel 1 goto no_numpy
call :say Python：%PY%

rem The tunnel's port here must be free: an old "云端连接" window may still hold it.
call :port_open 57120
if not errorlevel 1 goto port_busy

if exist "%SSH_EXIT%" del "%SSH_EXIT%"
call :say 正在连接云端（%REMOTE_USER%@%REMOTE_HOST%，端口 %REMOTE_PORT%）…
call :start_window :ssh_window SSH_PID
if not defined SSH_PID goto window_failed
call :say 等待云端连接（最多 120 秒）…
powershell -NoProfile -Command "$end=(Get-Date).AddSeconds(120); while((Get-Date) -lt $end){ if(Test-Path -LiteralPath $env:SSH_EXIT){exit 2}; $c=New-Object Net.Sockets.TcpClient; try{$c.Connect('127.0.0.1',57120); $c.Close(); exit 0}catch{}; Start-Sleep -Milliseconds 500 }; exit 1"
if errorlevel 2 goto ssh_ended
if errorlevel 1 goto ssh_timeout
call :say 已连上云端。服务器第一次启动 CARLA 约需 1 分钟，“云端连接”窗口显示“就绪”后即可运行。

call :say 启动 CarSim 服务…
call :start_window :service_window SVC_PID
if not defined SVC_PID goto window_failed

call :say 打开界面。关闭界面时，“云端连接”和“CarSim 服务”窗口会自动关闭；这个窗口请不要关。
start "" /wait "%EXE%" --auto-connect
call :say 界面已关闭，退出码 %errorlevel%
call :stop_windows
endlocal
exit /b 0

rem ---- errors ----------------------------------------------------------------
:not_unzipped
call :say [错误] 找不到启动包里的文件（ssh\remote_key、carla_cosim_studio.exe 或 service\carsim_service.py）。
call :say 请把整个压缩包解压（右键 → 全部解压缩），再双击解压出来的“启动远程仿真.bat”。
goto fail

:no_ssh
call :say [错误] 找不到 Windows 自带的 SSH 客户端（ssh.exe）。
call :say 请打开“设置 → 应用 → 可选功能 → 添加功能”，安装“OpenSSH 客户端”，然后再双击。
goto fail

:no_python
call :say [错误] 找不到 Python。请安装 64 位 Python 3（python.org，安装时勾选 Add python.exe to PATH），
call :say 或在本脚本开头的 CARSIM_PYTHON 填 python.exe 的完整路径。
goto fail

:no_numpy
call :say [错误] Python（%PY%）里没有 numpy，CarSim 服务需要它。请打开“命令提示符”运行：
call :say     %PY% -m pip install numpy
call :say 或在本脚本开头的 CARSIM_PYTHON 填已经装好 numpy 的 python.exe 的完整路径（例如虚拟环境里的）。
goto fail

:port_busy
call :say [错误] 这台电脑的端口 57120 已被占用：上一次的“云端连接”窗口可能还开着。
call :say 请关掉之前的“云端连接”“CarSim 服务”窗口和界面后再双击（还不行就重启电脑）。
goto fail

:window_failed
call :say [错误] 没能打开新窗口（需要 Windows 自带的 PowerShell）。
call :stop_windows
goto fail

:ssh_ended
call :say [错误] 没能连上云端，原因见“云端连接”窗口里的提示（常见问题见使用指南）。
call :say 看完后在这里按任意键，关闭“云端连接”窗口。
pause >nul
call :stop_windows
exit /b 1

:ssh_timeout
call :say [错误] 120 秒内没有连上云端：网络不通，或云服务器没有响应。请看“云端连接”窗口里的提示。
call :say 按任意键关闭“云端连接”窗口。
pause >nul
call :stop_windows
exit /b 1

:fail
call :say （记录：%LOG%）
pause
exit /b 1

rem ---- helpers ---------------------------------------------------------------
:say
rem Shows a message and adds it to the log.
echo(%*
>> "%LOG%" echo(%*
exit /b 0

:port_open
rem errorlevel 0 when something on this computer accepts connections on port %1.
powershell -NoProfile -Command "$c=New-Object Net.Sockets.TcpClient; try{$c.Connect('127.0.0.1',%1); $c.Close(); exit 0}catch{exit 1}"
exit /b %errorlevel%

:start_window
rem Runs this script with %1 in a new console window and puts that window's
rem process id into the variable %2 (cmd's "start" does not give it).
set "%2="
set "CC_ARG=%1"
for /f %%P in ('powershell -NoProfile -Command "$q=[char]34; (Start-Process -FilePath $env:ComSpec -ArgumentList ('/s /c '+$q+$q+$env:SELF+$q+' '+$env:CC_ARG+$q) -PassThru).Id"') do set "%2=%%P"
exit /b 0

:stop_windows
rem Closes the windows this script started, with what runs in them: by process
rem id, checked against the command line (an id can be reused by another
rem program), never by program name.
set "CC_PIDS=%SVC_PID% %SSH_PID%"
powershell -NoProfile -Command "foreach($p in ($env:CC_PIDS -split ' ')){ if($p){ $w=Get-CimInstance Win32_Process -Filter ('ProcessId='+$p); if($w -and $w.CommandLine -match ':(ssh|service)_window'){ taskkill.exe /PID $p /T /F | Out-Null } } }"
if exist "%SSH_EXIT%" del "%SSH_EXIT%"
exit /b 0

rem ---- the two windows ----------------------------------------------------------
:ssh_window
title 云端连接
echo 正在连接云端 %REMOTE_USER%@%REMOTE_HOST% …（这里显示云端的进度；关闭界面时这个窗口会自动关闭）
cd /d "%HERE%ssh"
rem -F none: settings in the user's own .ssh\config do not apply. LogLevel ERROR:
rem no line for each connection tried while the backend is still starting.
"%SSH%" -F none -i remote_key -p %REMOTE_PORT% -o UserKnownHostsFile=known_hosts -o StrictHostKeyChecking=yes -o BatchMode=yes -o IdentitiesOnly=yes -o ServerAliveInterval=15 -o ExitOnForwardFailure=yes -o LogLevel=ERROR -T -L 57120:127.0.0.1:57120 -L 57121:127.0.0.1:57121 %REMOTE_USER%@%REMOTE_HOST%
set "RC=%errorlevel%"
> "%SSH_EXIT%" echo %RC%
echo(
echo 与云端的连接已断开（ssh 退出码 %RC%）。原因见上面的提示；常见问题见使用指南。
pause
exit /b %RC%

:service_window
title CarSim 服务
cd /d "%HERE%service"
%PY% -u carsim_service.py
set "RC=%errorlevel%"
echo(
echo CarSim 服务已退出（退出码 %RC%）。
pause
exit /b %RC%
