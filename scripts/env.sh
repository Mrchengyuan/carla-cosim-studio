# Shared settings for the Linux scripts. Every value can be overridden by
# exporting it before running a script, e.g.  CARLA_ROOT=/opt/carla ./start_studio.sh
# Paths default to the layout described in docs/Ubuntu使用指南.md.
COSIM_ROOT="${COSIM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CARLA_ROOT="${CARLA_ROOT:-$COSIM_ROOT/CARLA_0.9.16}"          # original (prebuilt) CARLA
CARLA_SRC="${CARLA_SRC:-$COSIM_ROOT/carla_src}"                # modified CARLA source tree
UE4_ROOT="${UE4_ROOT:-$HOME/UnrealEngine_4.26}"                # CARLA's UE4 fork
STUDIO_BIN="${STUDIO_BIN:-$COSIM_ROOT/cosim_gui/build/carla_cosim_studio}"
CARLA_PORT="${CARLA_PORT:-2000}"                                # original CARLA
CARLA_MOD_PORT="${CARLA_MOD_PORT:-3000}"                        # modified CARLA
# Python that runs the backend: first one found wins.
if [ -z "$COSIM_PYTHON" ]; then
  for p in "$COSIM_ROOT/venv_build/bin/python" "$COSIM_ROOT/venv/bin/python" "$(command -v python3)"; do
    [ -x "$p" ] && { COSIM_PYTHON="$p"; break; }
  done
fi
# Optional HTTP proxy for downloads (build scripts only), e.g. http://127.0.0.1:7890
PROXY="${PROXY:-}"
export COSIM_ROOT CARLA_ROOT CARLA_SRC UE4_ROOT STUDIO_BIN CARLA_PORT CARLA_MOD_PORT COSIM_PYTHON PROXY
