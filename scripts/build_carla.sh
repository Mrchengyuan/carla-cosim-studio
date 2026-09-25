#!/bin/bash
# Build the modified CARLA 0.9.16 in $CARLA_SRC (needs build_ue4.sh first).
#   build_carla.sh            all steps
#   build_carla.sh <step>...  only some: source patch content pythonapi editor
# Steps:
#   source     clone CARLA 0.9.16 (tag) into $CARLA_SRC
#   patch      apply carla_patches/*.patch (skipped if already applied)
#   content    download + extract the 0.9.16 maps/vehicles (~21.6 GB)
#   pythonapi  make PythonAPI -> PythonAPI/carla/dist/carla-0.9.16-*.whl
#   editor     make CarlaUE4Editor (the Unreal plugin with our changes)
source "$(dirname "$0")/env.sh"
set -e
STEPS=${*:-source patch content pythonapi editor}
[ -n "$PROXY" ] && export http_proxy=$PROXY https_proxy=$PROXY HTTP_PROXY=$PROXY HTTPS_PROXY=$PROXY
export UE4_ROOT
for step in $STEPS; do
  case $step in
  source)
    if [ ! -d "$CARLA_SRC/.git" ]; then
      echo ">>> 克隆 CARLA 0.9.16 源码"
      git ${PROXY:+-c http.proxy=$PROXY} clone --depth 1 -b 0.9.16 https://github.com/carla-simulator/carla.git "$CARLA_SRC"
    else echo ">>> 源码已存在：$CARLA_SRC"; fi ;;
  patch)
    # File by file: a tree patched with an older version of a patch gets just
    # the files that are new in it.
    cd "$CARLA_SRC"
    for p in "$COSIM_ROOT"/carla_patches/*.patch; do
      for f in $(grep "^+++ b/" "$p" | cut -c7- | tr -d "\r"); do
        if git apply --reverse --check --include="$f" "$p" 2>/dev/null; then echo ">>> 已打过：$f"
        else git apply --include="$f" "$p" && echo ">>> 已应用：$f"; fi
      done
    done ;;
  content)
    D="$CARLA_SRC/Unreal/CarlaUE4/Content/Carla"
    if [ -f "$D/.version" ]; then echo ">>> 资源已存在：$D"; continue; fi
    ID=$(grep "^0.9.16:" "$CARLA_SRC/Util/ContentVersions.txt" | awk "{print \$2}")
    URL=https://carla-assets.s3.us-east-005.backblazeb2.com/$ID.tar.gz
    mkdir -p "$D"; cd "$CARLA_SRC"
    echo ">>> 下载资源 $ID（约 21.6 GB）"
    if command -v aria2c >/dev/null; then
      aria2c ${PROXY:+--all-proxy=$PROXY} -x16 -s16 -k16M --file-allocation=none -c -o content.tar.gz "$URL"
    else
      curl -L ${PROXY:+-x $PROXY} -C - -o content.tar.gz "$URL"
    fi
    echo ">>> 解压到 $D"; tar -xzf content.tar.gz -C "$D" && rm content.tar.gz && echo "$ID" > "$D/.version" ;;
  pythonapi)
    cd "$CARLA_SRC"; echo ">>> make PythonAPI（用当前激活的 python3）"; make PythonAPI
    ls PythonAPI/carla/dist/*.whl ;;
  editor)
    cd "$CARLA_SRC"; echo ">>> make CarlaUE4Editor"; make CarlaUE4Editor ;;
  *) echo "未知步骤 $step"; exit 1 ;;
  esac
done
echo ">>> 完成。启动改版服务器：scripts/carla_mod_server.sh"
