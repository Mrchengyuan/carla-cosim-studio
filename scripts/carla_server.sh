#!/bin/bash
# Original CARLA 0.9.16 (prebuilt), off-screen, on $CARLA_PORT.
#   carla_server.sh            off-screen (view it in the GUI "实时画面" page)
#   carla_server.sh --window   with the CARLA spectator window
source "$(dirname "$0")/env.sh"
[ -x "$CARLA_ROOT/CarlaUE4.sh" ] || { echo "找不到 $CARLA_ROOT/CarlaUE4.sh，请设置 CARLA_ROOT"; exit 1; }
cd "$CARLA_ROOT"
if [ "$1" = "--window" ]; then
  exec ./CarlaUE4.sh -nosound -carla-rpc-port=$CARLA_PORT
else
  exec ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=$CARLA_PORT
fi
