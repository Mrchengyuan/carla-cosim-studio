#!/bin/bash
# One-click start: CARLA (if not running) + CARLA CoSim Studio.
#   start_studio.sh          original CARLA 0.9.16
#   start_studio.sh mod      modified CARLA with the external-dynamics API
# Closing the GUI also stops the CARLA server this script started.
source "$(dirname "$0")/env.sh"
source "$(dirname "$0")/carla_stop_lib.sh"
MODE=${1:-stock}
# Started from a desktop icon there is no terminal: keep a log to see what happened.
LOG="${XDG_CACHE_HOME:-$HOME/.cache}/carla_cosim_studio/launch_$MODE.log"
mkdir -p "$(dirname "$LOG")" && : > "$LOG" && exec > >(tee -a "$LOG") 2>&1
echo "$(date '+%F %T') start_studio.sh $MODE"
if [ "$MODE" = "mod" ]; then
  PORT=$CARLA_MOD_PORT; SESSION=carla_mod; NAME="改版 CARLA"; TIMEOUT=900; KIND=mod
  START="bash '$COSIM_ROOT/scripts/carla_mod_server.sh'"
else
  PORT=$CARLA_PORT; SESSION=carla_server; NAME="原版 CARLA"; TIMEOUT=300; KIND=stock
  START="bash '$COSIM_ROOT/scripts/carla_server.sh'"
fi
listening() { ss -ltn | grep -q ":$PORT "; }
notify() { command -v zenity >/dev/null && zenity "$@" 2>/dev/null; }
[ -x "$STUDIO_BIN" ] || { notify --error --text="找不到界面程序 $STUDIO_BIN，请先编译 cosim_gui"; echo "missing $STUDIO_BIN"; exit 1; }

STARTED=0
if ! listening; then
  # Record when the server ends (stop.log says whether a script stopped it) and
  # keep its own output: a crash of CARLA itself is explained there.
  CARLA_LOG="${LOG%.log}_carla.log"
  [ -f "$CARLA_LOG" ] && mv -f "$CARLA_LOG" "$CARLA_LOG.prev"
  tmux new -d -s "$SESSION" "$START > '$CARLA_LOG' 2>&1; echo \"\$(date '+%F %T') $NAME 已退出，退出码 \$?（CARLA 的输出：$CARLA_LOG）\" >> '$LOG'"
  STARTED=1
  (
    # Closing the progress window must not end this wait: CARLA would then be
    # stopped as "failed to start" while it is still loading.
    trap '' PIPE
    for i in $(seq 1 $TIMEOUT); do
      listening && { sleep 3; echo 100 2>/dev/null; exit 0; }
      tmux has-session -t "$SESSION" 2>/dev/null || exit 1
      echo "# 正在启动 $NAME（端口 $PORT），已等待 $i 秒 ..." 2>/dev/null
      sleep 1
    done
    exit 1
  ) | { if command -v zenity >/dev/null; then zenity --progress --pulsate --auto-close --no-cancel \
          --title="CARLA CoSim Studio" --text="正在启动 $NAME ..." --width=420 2>/dev/null; else cat >/dev/null; fi; }
  if ! listening; then
    notify --error --title="CARLA CoSim Studio" --width=420 --text="$NAME 没有启动成功。\n启动记录：$LOG"
    echo "$NAME failed to start"
    stop_carla_server "$KIND" "端口 $PORT 没有打开（启动失败）"
    exit 1
  fi
fi

"$STUDIO_BIN" --python "$COSIM_PYTHON" --backend-dir "$COSIM_ROOT/carsim_carla_bridge" \
  --carla-port "$PORT" --auto-connect

if [ "$STARTED" = 1 ]; then
  stop_carla_server "$KIND" "界面已关闭"
fi
