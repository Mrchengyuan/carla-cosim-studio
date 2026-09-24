@echo off
rem Put "CARLA CoSim Studio" shortcuts on the desktop.
chcp 65001 >nul
set "HERE=%~dp0"
set "ICON=%~dp0..\..\cosim_gui\icons\studio.ico"
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$d=[Environment]::GetFolderPath('Desktop'); $s=New-Object -ComObject WScript.Shell;" ^
  "foreach($x in @(@('CARLA CoSim Studio',''),@('CARLA CoSim Studio (mod)','mod'))){" ^
  " $l=$s.CreateShortcut((Join-Path $d ($x[0]+'.lnk'))); $l.TargetPath='%HERE%start_studio.bat'; $l.Arguments=$x[1];" ^
  " $l.WorkingDirectory='%HERE%'; $l.WindowStyle=7; if(Test-Path '%ICON%'){$l.IconLocation='%ICON%'}; $l.Save() }" ^
  "$l=$s.CreateShortcut((Join-Path $d 'Stop CARLA.lnk')); $l.TargetPath='%HERE%stop_carla.bat'; $l.Save()"
echo 已在桌面创建快捷方式
timeout /t 3 >nul
