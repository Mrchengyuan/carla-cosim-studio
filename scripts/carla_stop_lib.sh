# Stops CARLA servers for real: killing the tmux session only ends the
# CarlaUE4.sh wrapper, the CARLA binary itself keeps running.
stop_carla_server() {   # $1 = stock | mod | all
  # Who stopped CARLA and why: a server that "just disappears" is then easy to explain.
  local log="${XDG_CACHE_HOME:-$HOME/.cache}/carla_cosim_studio/stop.log"
  mkdir -p "$(dirname "$log")" && echo "$(date '+%F %T') stop $1 by ${0##*/}${2:+: $2}" >> "$log"
  case "$1" in
    stock|all) tmux kill-session -t carla_server 2>/dev/null
               pkill -TERM -f "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" ;;&
    mod|all)   tmux kill-session -t carla_mod 2>/dev/null
               pkill -TERM -f "Binaries/Linux/UE4Editor .*CarlaUE4.uproject" ;;
  esac
  for i in $(seq 1 20); do
    pgrep -f "CarlaUE4-Linux-Shipping|UE4Editor .*CarlaUE4.uproject" >/dev/null || return 0
    sleep 0.5
  done
  pkill -KILL -f "CarlaUE4-Linux-Shipping|UE4Editor .*CarlaUE4.uproject"
}
