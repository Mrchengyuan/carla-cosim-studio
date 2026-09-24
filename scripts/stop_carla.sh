#!/bin/bash
# Stop every CARLA server started by these scripts and free the GPU.
source "$(dirname "$0")/carla_stop_lib.sh"
stop_carla_server all
command -v zenity >/dev/null && zenity --info --title="CARLA CoSim Studio" --width=320 --text="CARLA 已关闭，显存已释放。" 2>/dev/null
echo "CARLA stopped"
