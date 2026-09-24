#!/bin/bash
# Modified CARLA (editor build, game mode) with rendering, off-screen.
# Mesh distance fields are disabled: in uncooked editor mode UE4 builds them
# on the fly and the renderer can read a half-built one and crash.
export UE4_ROOT=~/UnrealEngine_4.26
cd ~/carla_carsim/carla_src/Unreal/CarlaUE4
"$UE4_ROOT/Engine/Binaries/Linux/UE4Editor" "$PWD/CarlaUE4.uproject" -game -RenderOffScreen -nosound \
  -carla-rpc-port=3000 -carla-streaming-port=0 -unattended -nosplash \
  -ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False \
  -ini:Engine:[/Script/Engine.RendererSettings]:r.DistanceFieldAO=False
