# Stops CARLA servers for real: killing the tmux session only ends the
# CarlaUE4.sh wrapper, the CARLA binary itself keeps running.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

_regex_escape() { printf '%s' "$1" | sed 's/[][\.*^$+?(){}|]/\\&/g'; }

stop_carla_server() {   # $1 = stock | mod | all, $2 = why (for stop.log)
  # Who stopped CARLA and why: a server that "just disappears" is then easy to explain.
  local log="${XDG_CACHE_HOME:-$HOME/.cache}/carla_cosim_studio/stop.log"
  mkdir -p "$(dirname "$log")" && echo "$(date '+%F %T') stop $1 by ${0##*/}${2:+: $2}" >> "$log"
  # stock / mod: only this installation's server of that kind (closing one
  # GUI must not kill the other kind, or a CARLA that is not ours).
  # all ("关闭 CARLA" icon): any CARLA server.
  local stock="$(_regex_escape "$CARLA_ROOT")/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
  local mod="UE4Editor $(_regex_escape "$CARLA_SRC")/Unreal/CarlaUE4/CarlaUE4\.uproject"
  local pat
  case "$1" in
    stock) tmux kill-session -t carla_server 2>/dev/null; pat="$stock" ;;
    mod)   tmux kill-session -t carla_mod 2>/dev/null; pat="$mod" ;;
    *)     tmux kill-session -t carla_server 2>/dev/null; tmux kill-session -t carla_mod 2>/dev/null
           pat="CarlaUE4-Linux-Shipping|UE4Editor .*CarlaUE4\.uproject" ;;
  esac
  pkill -TERM -f "$pat"
  for i in $(seq 1 20); do
    pgrep -f "$pat" >/dev/null || return 0
    sleep 0.5
  done
  pkill -KILL -f "$pat"
}
