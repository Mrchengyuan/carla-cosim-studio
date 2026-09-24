#!/bin/bash
# Desktop launcher: start CARLA (if not running), then CARLA CoSim Studio.
#   start_studio.sh          original CARLA 0.9.16 (port 2000)
#   start_studio.sh mod      modified CARLA with the external-dynamics API (port 3000)
# Closing the GUI also stops the CARLA server this script started.
MODE=${1:-stock}
ROOT=~/carla_carsim
if [ "$MODE" = "mod" ]; then
  PORT=3000; SESSION=carla_mod; NAME="改版 CARLA"; TIMEOUT=900
  START="bash $ROOT/scripts/carla_mod_server.sh"
else
  PORT=2000; SESSION=carla_server; NAME="原版 CARLA"; TIMEOUT=300
  START="cd $ROOT/CARLA_0.9.16 && ./CarlaUE4.sh -RenderOffScreen -nosound"
fi
listening() { ss -ltn | grep -q ":$PORT "; }
source "$ROOT/scripts/carla_stop_lib.sh"
KIND=$([ "$MODE" = "mod" ] && echo mod || echo stock)

STARTED=0
if ! listening; then
  tmux new -d -s "$SESSION" "$START"
  STARTED=1
  (
    for i in $(seq 1 $TIMEOUT); do
      listening && { sleep 3; echo 100; exit 0; }
      tmux has-session -t "$SESSION" 2>/dev/null || exit 1
      echo "# 正在启动 $NAME（端口 $PORT），已等待 $i 秒 ..."
      sleep 1
    done
    exit 1
  ) | zenity --progress --pulsate --auto-close --no-cancel --title="CARLA CoSim Studio" \
            --text="正在启动 $NAME ..." --width=420
  if ! listening; then
    zenity --error --title="CARLA CoSim Studio" --width=420 \
      --text="$NAME 没有启动成功。\n可以在终端执行 tmux attach -t $SESSION 查看原因。"
    stop_carla_server "$KIND"
    exit 1
  fi
fi

"$ROOT/cosim_gui/build/carla_cosim_studio" --python "$ROOT/venv_build/bin/python" \
  --carla-port "$PORT" --auto-connect

if [ "$STARTED" = 1 ]; then
  stop_carla_server "$KIND"
fi
