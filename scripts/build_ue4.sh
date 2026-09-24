#!/bin/bash
# Build CARLA's UE4 fork (4.26) into $UE4_ROOT. Takes ~1 h on 32 cores, ~95 GB.
# Needs a GitHub account linked to Epic Games (see docs/Ubuntu使用指南.md).
#   build_ue4.sh            clone (if needed) + Setup + GenerateProjectFiles + make
#   build_ue4.sh --no-clone reuse an existing $UE4_ROOT
source "$(dirname "$0")/env.sh"
set -e
if [ ! -d "$UE4_ROOT/Engine" ]; then
  [ "$1" = "--no-clone" ] && { echo "$UE4_ROOT 不存在"; exit 1; }
  echo ">>> 克隆 CARLA 定制版 UE4（会提示输入 GitHub 用户名和 Personal Access Token）"
  git ${PROXY:+-c http.proxy=$PROXY} clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git "$UE4_ROOT"
fi
cd "$UE4_ROOT"
[ -n "$PROXY" ] && export http_proxy=$PROXY https_proxy=$PROXY HTTP_PROXY=$PROXY HTTPS_PROXY=$PROXY
echo ">>> Setup.sh：下载约 12 GB 依赖和编译工具链"
# Setup.sh may ask whether to overwrite modified files; answer "no". Do not
# run it under "set -o pipefail": 'yes' exits non-zero when the pipe closes.
yes n | ./Setup.sh ${PROXY:+--proxy=$PROXY} --threads=16 || true
[ -f Engine/Build/OneTimeSetupPerformed ] || { echo "Setup.sh 没有成功，请检查网络 / 代理"; exit 1; }
echo ">>> GenerateProjectFiles.sh"
./GenerateProjectFiles.sh
echo ">>> make（不要加 -j，UE 会自动用满所有核心）"
make
echo ">>> UE4 编译完成：$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
