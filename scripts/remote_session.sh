#!/bin/bash
# Remote mode, server side: the command sshd runs for the restricted key of
# the Windows package (install_remote_key.sh, docs/远程使用指南.md). What it
# prints shows in the laptop's "云端连接" window.
#   1. Starts the modified CARLA like "start_studio.sh mod" (tmux session
#      carla_mod) unless something listens on $CARLA_MOD_PORT, and waits for
#      the port with ss, never by connecting (CARLA crashes after a few
#      hundred connects).
#   2. Runs the backend on 127.0.0.1:57120 (GUI) and 57121 (CarSim service);
#      the laptop reaches both through the SSH tunnel. Whenever the backend
#      exits it is started again (exit code 3: the GUI's "重启后端").
#   3. Ends with the SSH session (stdin closed, SIGHUP / SIGTERM, Ctrl+C) and
#      stops the backend: SIGTERM, SIGKILL after 10 s. CARLA keeps running,
#      so the next session is ready at once (stop_carla.sh stops it).
# A new session replaces a running one (the laptop reconnected). Run by hand
# for testing it works the same way; Ctrl+D or Ctrl+C ends it.
# Log: ~/.cache/carla_cosim_studio/remote_backend.log (the one before: .prev).
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/carla_stop_lib.sh"
GUI_PORT=57120      # the same ports as permitopen in the key's line and in 启动远程仿真.bat
CARSIM_PORT=57121
PORT=$CARLA_MOD_PORT
BRIDGE="$COSIM_ROOT/carsim_carla_bridge"
STATE="${XDG_CACHE_HOME:-$HOME/.cache}/carla_cosim_studio"
LOG="$STATE/remote_backend.log"
CARLA_LOG="$STATE/remote_carla.log"
PIDFILE="$STATE/remote_session.pid"
BPIDFILE="$STATE/remote_backend.pid"
mkdir -p "$STATE"

# The laptop may be gone already: a line it can no longer get must not end
# this script before the backend is stopped. (A handler, not an ignored
# signal: the programs started from here get the default again.)
trap ':' PIPE
say() { echo "$*" 2>/dev/null; echo "$(date '+%F %T') $*" >> "$LOG"; }
log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

# Waits about a second; false once the SSH session has ended (stdin closed).
wait_tick() { read -r -t 1 _ 2>/dev/null; [ $? -ne 1 ]; }

# $1 is a live process whose command line contains $2 (a pid file can be old).
is_ours() {
  [ -n "$1" ] && [ "$1" != "$$" ] && [ -r "/proc/$1/cmdline" ] &&
    tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -qF -- "$2"
}

stop_pid() {  # SIGTERM, SIGKILL after 10 s
  kill -TERM "$1" 2>/dev/null || return 0
  for _ in $(seq 1 100); do
    kill -0 "$1" 2>/dev/null || return 0
    sleep 0.1
  done
  log "后端 10 秒内没有退出，强制结束（PID $1）"
  kill -KILL "$1" 2>/dev/null
}

BPID=
cleanup() {
  [ -n "$CLEANED" ] && return
  CLEANED=1
  if [ -n "$BPID" ] && kill -0 "$BPID" 2>/dev/null; then
    log "停止后端（PID $BPID）"
    stop_pid "$BPID"
  fi
  [ -n "$BPID" ] && [ "$(cat "$BPIDFILE" 2>/dev/null)" = "$BPID" ] && rm -f "$BPIDFILE"
  [ "$(cat "$PIDFILE" 2>/dev/null)" = "$$" ] && rm -f "$PIDFILE"
  [ -n "$STARTED_SESSION" ] && log "远程会话结束（PID $$）"
}
trap cleanup EXIT
trap 'exit 0' HUP TERM INT

# --- a session that is still running is replaced -----------------------------
exec 9>>"$STATE/remote_session.lock"
if ! flock -w 60 9; then
  echo "[错误] 服务器上另一个连接正在启动，请稍后再试" 2>/dev/null
  exit 1
fi
OLD=$(cat "$PIDFILE" 2>/dev/null)
if is_ours "$OLD" remote_session.sh; then
  echo "云端还有上一次的连接，先结束它…" 2>/dev/null
  kill -TERM "$OLD" 2>/dev/null
  for _ in $(seq 1 150); do   # it stops its backend first (at most 10 s)
    is_ours "$OLD" remote_session.sh || break
    sleep 0.1
  done
  is_ours "$OLD" remote_session.sh && kill -KILL "$OLD" 2>/dev/null
fi
OLDB=$(cat "$BPIDFILE" 2>/dev/null)   # left behind by a session that was killed
is_ours "$OLDB" backend_server.py && stop_pid "$OLDB"
echo $$ > "$PIDFILE"
exec 9>&-
[ -f "$LOG" ] && mv -f "$LOG" "$LOG.prev"
STARTED_SESSION=1
log "远程会话开始（PID $$${SSH_CLIENT:+，来自 ${SSH_CLIENT%% *}}${OLD:+，替换 PID $OLD}）"
say "已连上云端服务器（$(hostname)）"

# --- CARLA (the checks of start_studio.sh) -----------------------------------
listening() { ss -ltn | grep -q ":$1 "; }
# Name of the program listening on port $1 (empty when it is another user's).
listener() { ss -ltnpH "sport = :$1" | sed -n 's/.*users:(("\([^"]*\)".*/\1/p' | sed -n 1p; }
# CARLA is known by the listener's name; otherwise (another user's process)
# it is asked its version once. Never probe CARLA with repeated connects.
is_carla() {
  case "$1" in CarlaUE4*|UE4Editor*) return 0 ;; esac
  "$COSIM_PYTHON" -c 'import carla, sys; c = carla.Client("localhost", int(sys.argv[1])); c.set_timeout(10); c.get_server_version()' "$PORT" >/dev/null 2>&1
}
if listening "$PORT"; then
  OWNER=$(listener "$PORT")
  if ! is_carla "$OWNER"; then
    say "[错误] 服务器上的端口 $PORT 被其他程序${OWNER:+（$OWNER）}占用，不是 CARLA。请告诉服务器管理员（scripts/env.sh 里的 CARLA_MOD_PORT）"
    exit 1
  fi
else
  STARTED=0
  if tmux has-session -t carla_mod 2>/dev/null; then
    log "改版 CARLA 正在由另一个启动器启动，等它启动好"
  else
    [ -f "$CARLA_LOG" ] && mv -f "$CARLA_LOG" "$CARLA_LOG.prev"
    # The settings go along explicitly: a tmux server that is already running
    # would start the session with its own (old) environment.
    ENVS="COSIM_ROOT='$COSIM_ROOT' CARLA_ROOT='$CARLA_ROOT' CARLA_SRC='$CARLA_SRC' UE4_ROOT='$UE4_ROOT' CARLA_PORT='$CARLA_PORT' CARLA_MOD_PORT='$CARLA_MOD_PORT'"
    tmux new -d -s carla_mod "env $ENVS bash '$COSIM_ROOT/scripts/carla_mod_server.sh' > '$CARLA_LOG' 2>&1; rc=\$?; echo \"\$(date '+%F %T') 改版 CARLA 已退出，退出码 \$rc（CARLA 的输出：$CARLA_LOG）\" >> '$LOG'" </dev/null
    STARTED=1
  fi
  say "正在启动 CARLA（首次约 1 分钟）…"
  # Up to an hour: the very first start compiles shaders (20-40 min).
  for ((i = 1; i <= 3600; i++)); do
    listening "$PORT" && break
    tmux has-session -t carla_mod 2>/dev/null || break
    (( i % 30 == 0 )) && say "  已等待 $i 秒…"
    wait_tick || exit 0   # the laptop has gone; CARLA goes on starting for the next session
  done
  if ! listening "$PORT"; then
    say "[错误] CARLA 没有启动成功（服务器上的记录：$CARLA_LOG）"
    [ "$STARTED" = 1 ] && stop_carla_server mod "端口 $PORT 没有打开（远程启动失败）"
    exit 1
  fi
  sleep 3   # listening, still finishing its start
fi
say "CARLA 已就绪"

# --- backend -------------------------------------------------------------------
BACKEND_ARGS=(--port "$GUI_PORT")
# Backends from before the remote CarSim service have no --carsim-port.
if (cd "$BRIDGE" && timeout 60 "$COSIM_PYTHON" backend_server.py --help 2>/dev/null </dev/null) | grep -q -- "--carsim-port"; then
  BACKEND_ARGS+=(--carsim-port "$CARSIM_PORT")
else
  log "backend_server.py 没有 --carsim-port 选项：Windows 上的 CarSim 服务连不上这个后端"
fi
start_backend() {
  (cd "$BRIDGE" && exec "$COSIM_PYTHON" -u backend_server.py "${BACKEND_ARGS[@]}") </dev/null >> "$LOG" 2>&1 &
  BPID=$!
  BSTART=$SECONDS
  READY=0
  echo "$BPID" > "$BPIDFILE"
  log "启动后端（PID $BPID）：backend_server.py ${BACKEND_ARGS[*]}"
}
# Ours is listening (not a stray program on the port).
backend_listening() { ss -ltnpH "sport = :$GUI_PORT" | grep -q "pid=$BPID,"; }

start_backend
FIRST=1
DELAY=1
while wait_tick; do
  if [ -n "$BPID" ]; then
    if kill -0 "$BPID" 2>/dev/null; then
      if [ "$READY" = 0 ] && backend_listening; then
        READY=1
        say "后端已就绪"
        [ -n "$FIRST" ] && say "就绪"
        FIRST=
      fi
      continue
    fi
    wait "$BPID"
    RC=$?
    BPID=
    if [ "$RC" = 3 ]; then
      say "后端正在重启…"
      RESTART_AT=$((SECONDS + 1))
    else
      # One that ends again right away (e.g. an error on start) is started less often.
      if (( SECONDS - BSTART < 30 )); then DELAY=$((DELAY * 2 > 30 ? 30 : DELAY * 2)); else DELAY=1; fi
      say "后端已退出（退出码 $RC），$DELAY 秒后重新启动（服务器上的记录：$LOG）"
      RESTART_AT=$((SECONDS + DELAY))
    fi
  elif (( SECONDS >= RESTART_AT )); then
    start_backend
  fi
done
exit 0   # the SSH session has ended: the EXIT trap stops the backend
