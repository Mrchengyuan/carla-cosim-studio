#!/bin/bash
# Modified CARLA without rendering: for testing the server-side API only.
export UE4_ROOT=~/UnrealEngine_4.26
cd ~/carla_carsim/carla_src/Unreal/CarlaUE4
"$UE4_ROOT/Engine/Binaries/Linux/UE4Editor" "$PWD/CarlaUE4.uproject" -game -nullrhi -nosound -carla-rpc-port=3000 -carla-streaming-port=0 -unattended -nosplash
