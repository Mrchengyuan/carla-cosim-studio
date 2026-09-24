#!/bin/bash
export UE4_ROOT=~/UnrealEngine_4.26
export http_proxy=http://127.0.0.1:7897 https_proxy=http://127.0.0.1:7897 HTTP_PROXY=http://127.0.0.1:7897 HTTPS_PROXY=http://127.0.0.1:7897 no_proxy=localhost,127.0.0.1
. ~/carla_carsim/venv_build/bin/activate
cd ~/carla_carsim/carla_src
echo "=== make CarlaUE4Editor $(date)"
make CarlaUE4Editor < /dev/null && echo "=== EDITOR_DONE $(date)" || echo "=== EDITOR_FAILED $(date)"
