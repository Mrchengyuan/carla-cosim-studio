# Ubuntu 使用指南

从一台全新的 Ubuntu 22.04 开始，到双击桌面图标就能用 CARLA CoSim Studio，逐步说明。
界面每个页面怎么操作见 [界面操作手册](界面操作手册.md)。

> 本指南的所有步骤都在 Ubuntu 22.04 + RTX 3090（驱动 560）上实际跑通过。

---

## 目录

1. [准备工作](#1-准备工作)
2. [获取代码](#2-获取代码)
3. [安装 CARLA（原版，推荐先用它）](#3-安装-carla原版推荐先用它)
4. [Python 环境](#4-python-环境)
5. [编译界面](#5-编译界面)
6. [启动](#6-启动)
7. [接入 CarSim](#7-接入-carsim)
8. [编译改版 CARLA（可选）](#8-编译改版-carla可选)
9. [命令行与强化学习](#9-命令行与强化学习)
10. [测试](#10-测试)
11. [常见问题](#11-常见问题)
12. [卸载与清理](#12-卸载与清理)

---

## 1. 准备工作

### 1.1 硬件和系统

| 项目 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 64 位（20.04 也可，未测试） |
| 显卡 | NVIDIA，显存 ≥ 6 GB（推荐 8 GB 以上）。多个相机同时采集时显存占用更高 |
| 驱动 | NVIDIA 专有驱动 ≥ 515（`nvidia-smi` 能正常显示即可） |
| 磁盘 | 只用原版 CARLA：约 30 GB。要编译改版 CARLA：再加约 150 GB（UE4 约 95 GB + CARLA 源码和资源约 31 GB + 余量） |
| 内存 | ≥ 16 GB；编译改版 CARLA 建议 ≥ 32 GB |

检查显卡驱动：
```bash
nvidia-smi          # 能看到显卡型号和驱动版本即可
```

### 1.2 安装系统软件包

```bash
sudo apt update
sudo apt install -y build-essential cmake git git-lfs curl wget unzip tmux aria2 \
    python3 python3-venv python3-pip python3-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev \
    libvulkan1 fonts-noto-cjk zenity imagemagick
```
各软件包的用途：
- `build-essential cmake`：编译界面。
- `libx11-dev … libgl1-mesa-dev`：界面用到的窗口和 OpenGL 库。
- `libvulkan1`：CARLA 渲染用。
- `fonts-noto-cjk`：界面的中文字体。**没有它，界面上的中文会显示成方块。**
- `tmux`：在后台运行 CARLA 服务器。
- `zenity`：一键启动时显示“正在启动 CARLA”的进度窗口。
- `aria2`：多线程下载 CARLA 资源（编译改版时用）。

### 1.3 网络（国内用户）

GitHub、CARLA 的下载服务器在国内可能很慢。有代理的话，先在终端里设置：
```bash
export http_proxy=http://127.0.0.1:7890 https_proxy=http://127.0.0.1:7890   # 换成你的代理地址
```
pip 可以用清华镜像：`pip install -i https://pypi.tuna.tsinghua.edu.cn/simple <包名>`。

---

## 2. 获取代码

推荐目录结构（脚本默认按这个结构找文件，不用改任何配置）：
```
~/carla-cosim-studio/              ← 本仓库
├── CARLA_0.9.16/                  ← 第 3 步解压的原版 CARLA
├── python_carsim_env/             ← CarSim 的 Python 接口
├── venv/                          ← 第 4 步创建的 Python 环境
├── carsim_carla_bridge/           ← 后端
├── cosim_gui/                     ← 界面
├── carla_patches/  scripts/  docs/
└── carla_src/                     ← （可选）第 8 步的改版 CARLA 源码
```

```bash
cd ~
git clone https://github.com/Mrchengyuan/carla-cosim-studio.git
cd carla-cosim-studio
git clone https://github.com/Mrchengyuan/python_carsim_env.git   # 原始仓库：https://github.com/dyZhou2001/python_carsim_env（原作者 dyZhou2001）
```
> 如果仓库是私有的，`git clone` 时会要求输入 GitHub 用户名和 Personal Access Token（在 GitHub → Settings → Developer settings → Personal access tokens 创建，勾选 `repo` 权限）。

目录放在别的地方也可以，脚本会自动识别自己所在的仓库位置。

---

## 3. 安装 CARLA（原版，推荐先用它）

原版 CARLA 不用编译，下载解压就能用。界面会自动使用“兼容模式”：画面上车身、转向、车轮转动完全同步，只是 `get_velocity()`、IMU 这类速度读数为 0，也没有悬架动画。想要这些，再做第 8 步。

### 3.1 下载和解压（约 8.3 GB，解压后 19 GB）
```bash
cd ~/carla-cosim-studio
mkdir CARLA_0.9.16
wget -c https://downloads.carlasim.com/Linux/CARLA_0.9.16.tar.gz      # 断了可以重新执行，会续传
tar -xzf CARLA_0.9.16.tar.gz -C CARLA_0.9.16
rm CARLA_0.9.16.tar.gz        # 解压完可以删掉安装包，省 8 GB
```
可选：额外地图 `AdditionalMaps_0.9.16.tar.gz`，下载后解压到 `CARLA_0.9.16/Import/`，再执行 `./ImportAssets.sh`。

### 3.2 试运行
```bash
cd ~/carla-cosim-studio/CARLA_0.9.16
./CarlaUE4.sh                   # 会弹出 CARLA 窗口，能看到城市画面说明正常；关掉窗口即退出
```
- 第一次启动约 10–30 秒。
- 不需要看 CARLA 自己的窗口时，用离屏模式（更省资源，界面的“实时画面”页照样能看到画面）：
  ```bash
  ./CarlaUE4.sh -RenderOffScreen
  ```
- 检查是否已经启动好：`ss -ltn | grep 2000`，能看到 2000 端口就说明好了。

---

## 4. Python 环境

后端需要 Python 3.10（Ubuntu 22.04 自带）和 CARLA 的 Python 包。用虚拟环境，不影响系统里的其他 Python：

```bash
cd ~/carla-cosim-studio
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl
pip install numpy pillow shapely networkx
```
检查安装：
```bash
python -c "import carla; print(carla.__file__)"
```
说明：
- CARLA 安装包自带 Python 3.10 / 3.11 / 3.12 的 wheel，请选和你的 Python 版本对应的那个（`python3 --version` 查看）。
- `shapely` 和 `networkx` 是“路线跟随”驾驶模式用的 CARLA 路径规划模块需要的。
- 路径规划模块本身（`agents`）不在 pip 包里，程序会自动从 `CARLA_0.9.16/PythonAPI/carla` 找。如果 CARLA 装在别处，设置环境变量：
  `export CARLA_PYTHONAPI=/你的路径/CARLA_0.9.16/PythonAPI/carla`

---

## 5. 编译界面

```bash
cd ~/carla-cosim-studio/cosim_gui
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```
- 大约 1 分钟。依赖库（ImGui、ImPlot、GLFW、json、图标字体）都在 `third_party/`，**不需要联网**。
- 生成 `build/carla_cosim_studio`，旁边的 `build/fonts/` 是图标字体，**两者要放在一起**。

---

## 6. 启动

### 6.1 方式一：桌面一键启动（推荐）

先安装桌面图标（只需一次）：
```bash
cd ~/carla-cosim-studio
./scripts/install_desktop_icons.sh
```
桌面和应用菜单里会出现 3 个图标：

| 图标 | 作用 |
|---|---|
| **CARLA CoSim Studio**（蓝） | 启动原版 CARLA，然后打开界面并自动连接 |
| **CARLA CoSim Studio（改版）**（绿） | 启动改版 CARLA（需要先完成第 8 步） |
| **关闭 CARLA**（红） | 关闭所有 CARLA，释放显存 |

使用：
1. 双击蓝色图标，会出现“正在启动原版 CARLA…”的进度窗口（10–30 秒）。
2. 界面自动打开并连上 CARLA，就可以操作了。
3. 用完**直接关闭界面窗口**，CARLA 会跟着自动关闭。

> 如果双击图标后提示“不受信任的应用程序”，右键图标 → **允许启动**。
> 一键启动默认用 `venv_build/bin/python` 或 `venv/bin/python`（先找到哪个用哪个）。

### 6.2 方式二：终端手动启动

开两个终端，或者用 tmux 把 CARLA 放到后台：
```bash
cd ~/carla-cosim-studio
# 1) 启动 CARLA（后台运行）
tmux new -d -s carla_server "./scripts/carla_server.sh"          # 需要 CARLA 窗口时加 --window
# 等端口出现
until ss -ltn | grep -q ":2000 "; do sleep 2; done; echo "CARLA 已就绪"
# 2) 启动界面
./cosim_gui/build/carla_cosim_studio --python venv/bin/python --auto-connect
```
关闭：关掉界面窗口，再执行 `./scripts/stop_carla.sh`。

界面程序的命令行参数：

| 参数 | 作用 |
|---|---|
| `--python <路径>` | 后端使用的 Python |
| `--backend-dir <目录>` | `carsim_carla_bridge` 目录（默认自动查找） |
| `--carla-port <端口>` | CARLA 端口（默认 2000） |
| `--auto-connect` | 启动后自动连接 CARLA |
| `--config <json>` | 启动时载入配置文件 |
| `--light` | 浅色主题 |
| `--size 2560x1440` | 窗口大小（默认 1680x1000） |
| `--scale 1.5` | 界面缩放（字体和控件一起放大，高分屏或截图用） |
| `--font <ttf/ttc>` | 指定中文字体 |

### 6.3 修改默认路径

所有脚本的路径都在 `scripts/env.sh` 里，也可以运行前用环境变量覆盖：
```bash
export CARLA_ROOT=/opt/CARLA_0.9.16          # 原版 CARLA 在别处
export COSIM_PYTHON=/home/me/miniconda3/envs/carla/bin/python
./scripts/start_studio.sh
```
可设置的变量：`CARLA_ROOT`、`CARLA_SRC`、`UE4_ROOT`、`STUDIO_BIN`、`COSIM_PYTHON`、`CARLA_PORT`（默认 2000）、`CARLA_MOD_PORT`（默认 3000）、`PROXY`。

---

## 7. 接入 CarSim

CarSim 的求解器通常在 **Windows** 上运行，所以真实 CarSim 联合仿真一般在 Windows 上做，见 [Windows 使用指南](Windows使用指南.md)。在 Ubuntu 上：

- **没有 Linux 版 CarSim**：在界面“CarSim 动力学”页勾选 **模拟 CarSim**，用内置的简单车辆模型跑通整条链路（驾驶、同步、采集都能测）。
- **有 Linux 版 CarSim 求解器**（`libcarsim.so` 和许可证）：`python_carsim_env` 本身支持 Linux，在“CarSim 动力学”页填 `.sim` 文件路径即可。`.sim` 里需要有 `SOFILE` 或 `PROGDIR` 指向求解器。

导出变量怎么配，见 [CarSim 导出变量清单](../carsim_carla_bridge/docs/CarSim导出变量清单.md)。

---

## 8. 编译改版 CARLA（可选）

改版 CARLA 增加了外部动力学接口：一帧一次下发完整状态，`get_velocity()` 和 IMU 读数是真实值，有悬架动画，切回 CARLA 物理时车速能接上。

> **需要约 150 GB 磁盘、2–3 小时**（32 核机器：UE4 约 1 小时，CARLA 依赖和编辑器约 30 分钟，下载资源视网速而定）。

### 8.1 关联 Epic 账号（只需一次）
CARLA 定制版 UE4 的代码只对关联了 Epic Games 的 GitHub 账号开放：
1. 登录 <https://www.epicgames.com>（免费注册）→ 账户 → **应用和账户** → GitHub → **连接**。
2. 到 GitHub 注册邮箱里接受 EpicGames 组织的邀请（或打开 <https://github.com/orgs/EpicGames/invitation>）。
3. 打开 <https://github.com/CarlaUnreal/UnrealEngine>，能看到代码就说明成功了（看到 404 就再等几分钟）。
4. 创建 Personal Access Token（classic，勾选 `repo`），克隆时密码处填它。

### 8.2 编译 UE4（约 1 小时，95 GB）
```bash
cd ~/carla-cosim-studio
PROXY=http://127.0.0.1:7890 ./scripts/build_ue4.sh      # 不需要代理就去掉 PROXY=...
```
脚本会依次做：克隆 UE4 到 `~/UnrealEngine_4.26` → `Setup.sh`（下载约 12 GB 依赖和编译器）→ `GenerateProjectFiles.sh` → `make`。
已经克隆过的话：`./scripts/build_ue4.sh --no-clone`。

### 8.3 编译改版 CARLA
```bash
cd ~/carla-cosim-studio
source venv/bin/activate                     # PythonAPI 会按当前 Python 版本编译
pip install -r https://raw.githubusercontent.com/carla-simulator/carla/0.9.16/PythonAPI/carla/requirements.txt
PROXY=http://127.0.0.1:7890 ./scripts/build_carla.sh
```
脚本依次执行以下 5 步，任何一步失败后都可以只重跑那一步，例如 `./scripts/build_carla.sh pythonapi editor`：

| 步骤 | 做什么 |
|---|---|
| `source` | 克隆 CARLA 0.9.16 源码到 `carla_src/` |
| `patch` | 打上 `carla_patches/` 里的两个补丁（外部动力学接口 + libpng 下载地址修复） |
| `content` | 下载并解压地图 / 车辆资源（约 21.6 GB） |
| `pythonapi` | 编译 LibCarla 和 Python 包，生成 `carla_src/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl` |
| `editor` | 编译 CARLA 的 Unreal 插件（包含我们的改动） |

编译完成后，把改版的 Python 包装进一个单独的环境：
```bash
python3 -m venv venv_build && source venv_build/bin/activate
pip install carla_src/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl numpy pillow shapely networkx
python -c "import carla; print(hasattr(carla.Vehicle, 'apply_external_state'))"    # 应输出 True
```

### 8.4 启动改版 CARLA
- 桌面双击绿色图标 **CARLA CoSim Studio（改版）**，或者
- 手动：`tmux new -d -s carla_mod "./scripts/carla_mod_server.sh"`（端口 3000）。

注意事项：
- **第一次启动要编译着色器，可能需要 20–40 分钟**，以后每次约 1–2 分钟。
- 改版 CARLA 是以“编辑器游戏模式”运行的，**第一次切换到某张地图也要现场编译，会很慢**。需要频繁切换地图时，可以打包成正式版：`cd carla_src && make package`（再需要 1–2 小时和约 20 GB 空间），打包版切换地图约 6 秒。
- 只测试接口、不需要画面时：`./scripts/carla_mod_server.sh --norender`。
- 验证改版接口：`venv_build/bin/python carsim_carla_bridge/tests/test_modified_carla.py --port 3000`，应该 11 项全部 PASS。

编译过程中我们遇到并已在脚本里处理的问题（供参考）：
- CARLA 0.9.16 的 `Setup.sh` 里 libpng 下载地址已失效 → `carla_patches/carla_0.9.16_linux_libpng_url_fix.patch`。
- UE4 `Setup.sh` 可能询问是否覆盖文件 → 脚本自动回答“否”。
- 编辑器模式下，网格距离场会在运行时现场生成，渲染线程可能读到一半生成的数据而崩溃 → 启动脚本关闭了 `r.GenerateMeshDistanceFields`。

---

## 9. 命令行与强化学习

不打开界面也能运行，配置和界面保存的 JSON 通用：
```bash
cd ~/carla-cosim-studio/carsim_carla_bridge
source ../venv/bin/activate
python run_cosim.py --mock --duration 20                      # 模拟 CarSim + 示例控制算法
python run_cosim.py --config ../cosim_config.json             # 用界面保存的配置
python run_cosim.py --sim /path/to/simfile.sim --controller controllers/my_controller.py   # 真实 CarSim + 你的控制算法
```
在自己的训练代码里使用（每个 `env.control_step()` 之后加两行）：
```python
from bridge import CarlaVehicleSync
sync = CarlaVehicleSync(world, vehicle, anchor_transform)      # 初始化一次
obs, r, done, info = env.control_step(action, inner_steps)
sync.sync(obs, env.t_current, frame_dt)
world.tick()
```
更多说明见 [桥接说明](../carsim_carla_bridge/README.md)。

---

## 10. 测试

CARLA 启动后，在 `carsim_carla_bridge` 目录执行：
```bash
python tests/test_coords.py                          # 坐标换算（不需要 CARLA）
python tests/test_backend.py                         # 界面后端全部命令，约 2 分钟
python tests/test_features.py                        # 驾驶模式 + 3 帧采集（测完自动删除）
python tests/test_modified_carla.py --port 3000      # 改版 CARLA 接口（需要改版 CARLA 和 venv_build）
```
界面自动演示：`./cosim_gui/build/carla_cosim_studio --tour /tmp/tour --auto-connect`（会依次操作每个页面并截图到 `/tmp/tour`）。

---

## 11. 常见问题

| 现象 | 原因和解决 |
|---|---|
| 界面中文显示成方块 | 缺中文字体：`sudo apt install fonts-noto-cjk`，或用 `--font` 指定字体 |
| 界面打不开，报 `GLFW ... Failed to open display` | 没有图形桌面（例如纯 SSH 登录）。请在机器的桌面环境里启动 |
| 状态栏“后端 未运行” | Python 路径不对或缺包。看 `carsim_carla_bridge/backend.log`；在“连接”页改 Python 解释器 |
| 连接 CARLA 超时 | CARLA 还没启动好（`ss -ltn \| grep 2000` 看端口），或端口填错（原版 2000，改版 3000） |
| 日志提示 `57100` 端口被占用 | 上次的后端还在：`pkill -f backend_server.py` |
| 路线跟随报 `No module named agents` | 找不到 CARLA 的路径规划模块：设置 `CARLA_PYTHONAPI`（见第 4 步） |
| 车停住不动 | ① 工具栏显示“已完成”、画面上方有黄色提示条：运行时长到了，车被停住；把“驾驶模式 → 运行时长”设为 0（一直运行）。② CARLA 自动驾驶在等红灯（视口上方提示“正在等红灯”），变绿后会自己走；也可以勾选忽略红绿灯。注意“路线跟随”不看红绿灯 |
| 连接时提示 `Version mismatch` | 用改版客户端连原版服务器（或反过来）时的正常提示，不影响使用 |
| 采集点“运行”被拒绝 | 没设停止条件，或预计大小超过磁盘剩余空间（需保留 10 GB），在“数据采集”页调整 |
| CARLA 关了但显存还被占 | 双击“关闭 CARLA”图标，或执行 `./scripts/stop_carla.sh` |
| 改版 CARLA 启动后崩溃 | 看 `carla_src/Unreal/CarlaUE4/Saved/Crashes/` 下最新的 `Diagnostics.txt`；确认用的是 `scripts/carla_mod_server.sh` 启动 |
| 显示器分辨率高、界面太小 | 在系统设置里调整缩放，界面会跟随系统缩放 |

---

## 12. 卸载与清理

```bash
./scripts/stop_carla.sh                                  # 先关掉 CARLA
rm ~/桌面/carla-*.desktop ~/.local/share/applications/carla-*.desktop   # 桌面图标（英文系统是 ~/Desktop）
rm -rf ~/carla-cosim-studio                              # 本仓库、原版 CARLA、Python 环境
rm -rf ~/UnrealEngine_4.26                               # 编译改版时的 UE4（约 95 GB）
```
