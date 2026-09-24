# Stops CARLA servers for real: killing the tmux session only ends the
# CarlaUE4.sh wrapper, the CARLA binary itself keeps running.
stop_carla_server() {   # $1 = stock | mod | all
  case "$1" in
    stock|all) tmux kill-session -t carla_server 2>/dev/null
               pkill -TERM -f "CARLA_0.9.16/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" ;;&
    mod|all)   tmux kill-session -t carla_mod 2>/dev/null
               pkill -TERM -f "Binaries/Linux/UE4Editor .*CarlaUE4.uproject" ;;
  esac
  for i in $(seq 1 20); do
    pgrep -f "CarlaUE4-Linux-Shipping|UE4Editor .*CarlaUE4.uproject" >/dev/null || return 0
    sleep 0.5
  done
  pkill -KILL -f "CarlaUE4-Linux-Shipping|UE4Editor .*CarlaUE4.uproject"
}
