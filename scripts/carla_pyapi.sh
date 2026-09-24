#!/bin/bash
export UE4_ROOT=~/UnrealEngine_4.26
export http_proxy=http://127.0.0.1:7897 https_proxy=http://127.0.0.1:7897 HTTP_PROXY=http://127.0.0.1:7897 HTTPS_PROXY=http://127.0.0.1:7897 no_proxy=localhost,127.0.0.1
. ~/carla_carsim/venv_build/bin/activate
cd ~/carla_carsim/carla_src
echo "=== make PythonAPI $(date)"
make PythonAPI < /dev/null && echo "=== PYTHONAPI_DONE $(date)" || echo "=== PYTHONAPI_FAILED $(date)"
