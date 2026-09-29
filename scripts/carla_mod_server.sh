#!/bin/bash
# Modified CARLA on $CARLA_MOD_PORT.
#   carla_mod_server.sh              off-screen rendering
#   carla_mod_server.sh --norender   no rendering at all (API tests only)
# The packaged build ($CARLA_MOD_ROOT, from "make package" in carla_src) is used
# when it is there: it starts in seconds. Otherwise the editor build (game mode).
# Editor build: mesh distance fields are disabled: uncooked, UE4 builds them
# on the fly and the renderer can read a half-built one and crash. The project
# goes by its physical path (cd -P): carla_stop_lib.sh finds the server by it.
source "$(dirname "$0")/env.sh"
if [ "$1" = "--norender" ]; then RENDER="-nullrhi"; else RENDER="-RenderOffScreen"; fi
if [ -x "$CARLA_MOD_ROOT/CarlaUE4.sh" ]; then
  cd "$CARLA_MOD_ROOT"
  exec ./CarlaUE4.sh $RENDER -nosound -carla-rpc-port=$CARLA_MOD_PORT -carla-streaming-port=0
fi
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
[ -x "$EDITOR" ] || { echo "找不到打包版 $CARLA_MOD_ROOT/CarlaUE4.sh，也找不到编辑器 $EDITOR，请设置 CARLA_MOD_ROOT 或 UE4_ROOT"; exit 1; }
cd -P "$CARLA_SRC/Unreal/CarlaUE4" || { echo "找不到 $CARLA_SRC/Unreal/CarlaUE4，请设置 CARLA_SRC（改版 CARLA 的源码目录）"; exit 1; }
exec "$EDITOR" "$PWD/CarlaUE4.uproject" -game $RENDER -nosound \
  -carla-rpc-port=$CARLA_MOD_PORT -carla-streaming-port=0 -unattended -nosplash \
  -ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False \
  -ini:Engine:[/Script/Engine.RendererSettings]:r.DistanceFieldAO=False
