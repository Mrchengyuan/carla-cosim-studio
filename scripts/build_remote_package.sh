#!/bin/bash
# Remote mode: builds the Windows package (docs/远程使用指南.md)
#   CARLA_CoSim_Studio_Remote/
#     启动远程仿真.bat  carla_cosim_studio.exe  fonts/fa-solid-900.ttf
#     cosim_studio_prefs.json  service/*.py  ssh/remote_key  ssh/known_hosts  使用说明.txt
# and its zip. The GUI is the MinGW build of this checkout (cosim_gui/build-win,
# built or brought up to date here), unless --exe gives a Windows build.
#   ssh-keyscan -p 32122 i.easy-ai.cloud > kh.txt     # on a computer OUTSIDE the server
#   scripts/build_remote_package.sh --known-hosts kh.txt
# Options (or the environment variable):
#   --key FILE          REMOTE_KEY           the private key from install_remote_key.sh
#                                            (default ~/.config/carla_cosim_studio/remote_key)
#   --known-hosts FILE  REMOTE_KNOWN_HOSTS   (the variable holds the lines themselves)
#   --out ZIP           REMOTE_PACKAGE       default ~/carla_cosim_remote_pkg/CARLA_CoSim_Studio_Remote.zip
#   --exe FILE          a Windows carla_cosim_studio.exe instead of building one
# The key and known_hosts go only into the package and the zip, which must be
# outside the repository: they are never committed. Send the zip privately.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
KEY="${REMOTE_KEY:-$HOME/.config/carla_cosim_studio/remote_key}"
KNOWN_FILE=
KNOWN="${REMOTE_KNOWN_HOSTS:-}"
OUT="${REMOTE_PACKAGE:-$HOME/carla_cosim_remote_pkg/CARLA_CoSim_Studio_Remote.zip}"
EXE=
fail() { echo "error: $*" >&2; exit 1; }
while [ $# -gt 0 ]; do
  case "$1" in
    --key) KEY="$2"; shift 2 ;;
    --known-hosts) KNOWN_FILE="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --exe) EXE="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) fail "unknown option $1 (see --help)" ;;
  esac
done
NAME=CARLA_CoSim_Studio_Remote
BAT="$COSIM_ROOT/scripts/windows/启动远程仿真.bat"
BRIDGE="$COSIM_ROOT/carsim_carla_bridge"
FONT="$COSIM_ROOT/cosim_gui/third_party/fonts/fa-solid-900.ttf"
REPO=$(readlink -f "$COSIM_ROOT")
inside_repo() { case "$(readlink -m "$1")/" in "$REPO"/*) return 0 ;; esac; return 1; }

# --- inputs ------------------------------------------------------------------
[ -f "$BAT" ] || fail "missing $BAT"
[ -r "$KEY" ] || fail "no private key $KEY (make it with scripts/install_remote_key.sh, or give --key)"
grep -q "BEGIN OPENSSH PRIVATE KEY" "$KEY" || fail "$KEY is not an OpenSSH private key"
inside_repo "$KEY" && fail "the key $KEY is inside the repository: keep it outside (never commit it)"
if [ -n "$KNOWN_FILE" ]; then
  [ -r "$KNOWN_FILE" ] || fail "cannot read $KNOWN_FILE"
  KNOWN=$(cat "$KNOWN_FILE")
fi
KNOWN=$(printf '%s\n' "$KNOWN" | tr -d '\r' | grep -v '^[[:space:]]*\(#\|$\)')
[ -n "$KNOWN" ] || fail "no known_hosts lines: give --known-hosts FILE (ssh-keyscan -p PORT HOST, run outside the server)"
# The launcher connects to the host and port set in the .bat.
HOST=$(sed -n 's/^set "REMOTE_HOST=\([^"]*\)".*/\1/p' "$BAT" | head -n 1)
SSHPORT=$(sed -n 's/^set "REMOTE_PORT=\([^"]*\)".*/\1/p' "$BAT" | head -n 1)
[ -n "$HOST" ] && [ -n "$SSHPORT" ] || fail "REMOTE_HOST / REMOTE_PORT not found in $BAT"
if ! printf '%s\n' "$KNOWN" | grep -q -e "^\[$HOST\]:$SSHPORT[ ,]" -e '^|1|'; then
  fail "the known_hosts lines are not for [$HOST]:$SSHPORT (the launcher connects there)"
fi
OUT=$(readlink -m "$OUT")
DEST=$(dirname "$OUT")
inside_repo "$OUT" && fail "$OUT is inside the repository: the package holds the private key, write it elsewhere"
case "$OUT" in *.zip) ;; *) fail "--out must end in .zip" ;; esac

# --- the GUI -------------------------------------------------------------------
if [ -z "$EXE" ]; then
  BUILD="$COSIM_ROOT/cosim_gui/build-win"
  command -v x86_64-w64-mingw32-g++-posix >/dev/null || fail "no MinGW (sudo apt install g++-mingw-w64-x86-64-posix)"
  if [ ! -f "$BUILD/CMakeCache.txt" ]; then
    cmake -S "$COSIM_ROOT/cosim_gui" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE="$COSIM_ROOT/cosim_gui/mingw-toolchain.cmake" || fail "cmake configure failed"
  fi
  nice -n 10 cmake --build "$BUILD" -j "${JOBS:-8}" || fail "the Windows build failed"
  EXE="$BUILD/carla_cosim_studio.exe"
fi
[ -f "$EXE" ] || fail "missing $EXE"
[ "$(head -c 2 "$EXE")" = MZ ] || fail "$EXE is not a Windows program"

# --- assemble ----------------------------------------------------------------------
umask 077   # the package holds the private key
mkdir -p "$DEST" || fail "cannot create $DEST"
STAGE=$(mktemp -d "$DEST/.$NAME.XXXXXX") || fail "cannot write in $DEST"
trap 'rm -rf "$STAGE"' EXIT
P="$STAGE/$NAME"
mkdir -p "$P/fonts" "$P/service" "$P/ssh"
# CRLF whatever the checkout did (cmd needs it; .gitattributes also says so).
sed 's/\r$//; s/$/\r/' "$BAT" > "$P/启动远程仿真.bat"
cp "$EXE" "$P/carla_cosim_studio.exe"
cp "$FONT" "$P/fonts/" || fail "missing $FONT"
# The GUI's defaults (App::Init in cosim_gui/src/app.cpp) with the remote ones:
# no local backend, the tunnel's port, the modified CARLA on the server.
cat > "$P/cosim_studio_prefs.json" <<'EOF'
{
  "remote_backend": true,
  "auto_start_backend": false,
  "backend_port": 57120,
  "carla_host": "localhost",
  "carla_port": 3000,
  "python": "python",
  "backend_dir": "",
  "last_config": "",
  "dark_theme": true
}
EOF
for f in carsim_service.py carsim_local.py mock_carsim.py; do
  if [ -f "$BRIDGE/$f" ]; then cp "$BRIDGE/$f" "$P/service/"; else echo "warning: $BRIDGE/$f is missing (package without it)" >&2; fi
done
cp "$KEY" "$P/ssh/remote_key"
printf '%s\n' "$KNOWN" > "$P/ssh/known_hosts"
# Notepad: UTF-8 with BOM, CRLF.
{ printf '\xef\xbb\xbf'; sed 's/$/\r/'; } > "$P/使用说明.txt" <<'EOF'
CARLA CoSim Studio 远程版（笔记本 + 云服务器）

这台电脑只运行 CarSim 和界面；CARLA、后端和你的控制算法在云服务器上运行。

第一次使用
1. 把整个压缩包解压（右键 → 全部解压缩），不要在压缩包里直接双击。
2. 这台电脑要有 64 位 Python 3.8 以上，并装好 numpy：py -3 -m pip install numpy
3. 在 CarSim 里按“CarSim 导出变量清单”设置导入 / 导出变量；
   在界面的“CarSim 动力学”页填 .sim 文件和 python_carsim_env 目录（这台电脑上的路径）。

每次使用
双击“启动远程仿真.bat”。会依次出现“云端连接”“CarSim 服务”两个窗口和界面；
“云端连接”窗口显示“就绪”后即可运行。关闭界面时另外两个窗口自动关闭。

运行记录保存在云服务器上（carsim_carla_bridge/runs/...）。
完整说明和常见问题：仓库里的 docs/远程使用指南.md
https://github.com/Mrchengyuan/carla-cosim-studio/blob/main/docs/远程使用指南.md

ssh 文件夹里是连接云服务器的钥匙：只能用来建立这条连接，但请不要发给别人。
更新启动包时，先删掉旧文件夹再解压新的。
EOF

# --- replace the old package, then zip it -------------------------------------------
rm -rf "$DEST/$NAME"
mv "$P" "$DEST/$NAME" || fail "cannot write $DEST/$NAME"
python3 - "$DEST" "$NAME" "$OUT.tmp" <<'EOF' || fail "zip failed"
import os, sys, zipfile
dest, name, out = sys.argv[1:]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:  # UTF-8 names: Windows shows them right
    for root, dirs, files in os.walk(os.path.join(dest, name)):
        dirs.sort()
        for f in sorted(files):
            path = os.path.join(root, f)
            z.write(path, os.path.relpath(path, dest))
EOF
mv -f "$OUT.tmp" "$OUT"
echo "package: $DEST/$NAME"
echo "zip:     $OUT ($(du -h "$OUT" | cut -f1))"
python3 -c 'import sys, zipfile; [print("  " + n) for n in zipfile.ZipFile(sys.argv[1]).namelist()]' "$OUT"
echo "The zip holds the private key: send it privately, never commit it."
