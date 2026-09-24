#!/bin/bash
# Stop every CARLA server started from the desktop launchers.
source ~/carla_carsim/scripts/carla_stop_lib.sh
stop_carla_server all
zenity --info --title="CARLA CoSim Studio" --width=320 --text="CARLA 已关闭，显存已释放。" 2>/dev/null
