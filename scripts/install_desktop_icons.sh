#!/bin/bash
# Put double-click launchers on the desktop and in the application menu.
source "$(dirname "$0")/env.sh"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
APPS="$HOME/.local/share/applications"
ICONS="$COSIM_ROOT/cosim_gui/icons"
mkdir -p "$DESK" "$APPS"
make_entry() {  # file name exec icon comment
  cat > "$1" <<EOT
[Desktop Entry]
Type=Application
Version=1.0
Name=$2
Comment=$5
Exec=$3
Icon=$4
Terminal=false
Categories=Science;Engineering;
StartupNotify=true
EOT
  chmod +x "$1"
}
for D in "$DESK" "$APPS"; do
  make_entry "$D/carla-cosim-studio.desktop" "CARLA CoSim Studio" "\"$COSIM_ROOT/scripts/start_studio.sh\"" \
    "$ICONS/studio.png" "启动原版 CARLA 并打开联合仿真界面"
  make_entry "$D/carla-cosim-studio-mod.desktop" "CARLA CoSim Studio（改版）" "\"$COSIM_ROOT/scripts/start_studio.sh\" mod" \
    "$ICONS/studio_mod.png" "启动改版 CARLA（外部动力学接口）并打开联合仿真界面"
  make_entry "$D/carla-stop.desktop" "关闭 CARLA" "\"$COSIM_ROOT/scripts/stop_carla.sh\"" \
    "$ICONS/stop.png" "关闭所有 CARLA 服务器，释放显存"
done
# GNOME: mark desktop launchers as trusted so they start on double-click.
for f in "$DESK"/carla-*.desktop; do gio set "$f" metadata::trusted true 2>/dev/null; done
echo "已安装到：$DESK 和 $APPS"
