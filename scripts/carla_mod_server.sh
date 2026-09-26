#!/bin/bash
# Modified CARLA (editor build, game mode) on $CARLA_MOD_PORT.
#   carla_mod_server.sh              off-screen rendering
#   carla_mod_server.sh --norender   no rendering at all (API tests only)
# Mesh distance fields are disabled: in uncooked editor mode UE4 builds them
# on the fly and the renderer can read a half-built one and crash.
source "$(dirname "$0")/env.sh"
EDITOR="$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
[ -x "$EDITOR" ] || { echo "找不到 $EDITOR，请设置 UE4_ROOT"; exit 1; }
cd "$CARLA_SRC/Unreal/CarlaUE4" || { echo "找不到 $CARLA_SRC/Unreal/CarlaUE4，请设置 CARLA_SRC（改版 CARLA 的源码目录）"; exit 1; }
if [ "$1" = "--norender" ]; then RENDER="-nullrhi"; else RENDER="-RenderOffScreen"; fi
exec "$EDITOR" "$PWD/CarlaUE4.uproject" -game $RENDER -nosound \
  -carla-rpc-port=$CARLA_MOD_PORT -carla-streaming-port=0 -unattended -nosplash \
  -ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False \
  -ini:Engine:[/Script/Engine.RendererSettings]:r.DistanceFieldAO=False
