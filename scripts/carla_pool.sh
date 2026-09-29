#!/bin/bash
# 私有端口上的一组原版 CARLA 服务器（强化学习的并行环境、测试用；不碰界面用的 2000 / 3000）。
#   carla_pool.sh start 2100 2200 ...   每个端口起一个（已经在监听的跳过），等到都在监听
#   carla_pool.sh stop  2100 2200 ...   只关这些端口的（按起它时记下的进程组，不按名字找）
#   carla_pool.sh status                记下的服务器和它们是不是还在监听
# 进程号记在 ~/.cache/carla_cosim_studio/pool/<端口>.pid，日志在同一目录的 <端口>.log。
# 不通过连接 RPC 端口来判断（连上再断开约 300 次 CARLA 0.9.16 就会段错误），看 /proc/net/tcp 的监听表。
source "$(dirname "$0")/env.sh"
POOL="${XDG_CACHE_HOME:-$HOME/.cache}/carla_cosim_studio/pool"
mkdir -p "$POOL"

listening() {   # $1 = 端口
  local hex; hex=$(printf '%04X' "$1")
  awk -v h="$hex" 'NR > 1 && $4 == "0A" { n = split($2, a, ":"); if (toupper(a[n]) == h) f = 1 } END { exit !f }' /proc/net/tcp
}

cmd="$1"; shift
case "$cmd" in
  start)
    [ $# -gt 0 ] || { echo "用法：$0 start 端口 [端口 ...]"; exit 1; }
    for p in "$@"; do
      if listening "$p"; then echo "端口 $p 已经在监听，跳过"; continue; fi
      CARLA_PORT=$p setsid nohup "$(dirname "$0")/carla_server.sh" > "$POOL/$p.log" 2>&1 < /dev/null &
      echo $! > "$POOL/$p.pid"
    done
    for p in "$@"; do
      for i in $(seq 1 120); do listening "$p" && break; sleep 1; done
      listening "$p" && echo "端口 $p 在监听" || { echo "端口 $p 120 s 内没有起来，日志：$POOL/$p.log"; exit 1; }
    done ;;
  stop)
    [ $# -gt 0 ] || { echo "用法：$0 stop 端口 [端口 ...]"; exit 1; }
    for p in "$@"; do
      pid=$(cat "$POOL/$p.pid" 2>/dev/null)
      if [ -z "$pid" ]; then echo "端口 $p：没有记录（不是这个脚本起的），不动"; continue; fi
      kill -TERM -- "-$pid" 2>/dev/null
      for i in $(seq 1 20); do listening "$p" || break; sleep 0.5; done
      if listening "$p"; then
        kill -KILL -- "-$pid" 2>/dev/null
        for i in $(seq 1 20); do listening "$p" || break; sleep 0.25; done   # 强制结束后端口也要过一会儿才关
      fi
      rm -f "$POOL/$p.pid"
      listening "$p" && echo "端口 $p 还在监听" || echo "端口 $p 已关闭"
    done ;;
  status)
    for f in "$POOL"/*.pid; do
      [ -e "$f" ] || { echo "没有记录的服务器"; break; }
      p=$(basename "$f" .pid)
      listening "$p" && echo "端口 $p：在监听（进程组 $(cat "$f")）" || echo "端口 $p：记录着但没有在监听"
    done ;;
  *) echo "用法：$0 start|stop|status [端口 ...]"; exit 1 ;;
esac
