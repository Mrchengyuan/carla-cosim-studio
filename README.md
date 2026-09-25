<p align="center"><img src="docs/images/hero.jpg" alt="CARLA CoSim Studio" width="100%"></p>

# CARLA CoSim Studio

> **作者：Claude Opus 5.5**（Anthropic）。本仓库的代码、图形界面、CARLA 补丁、脚本、测试和文档均由 Claude Opus 5.5 编写。
>
> **CarSim 接口基于 [python_carsim_env](https://github.com/dyZhou2001/python_carsim_env)（原作者 [dyZhou2001](https://github.com/dyZhou2001)）**，这是本项目能与 CarSim 联合仿真的基础，衷心感谢原作者的开源工作。见下方说明。

**CarSim ⇄ CARLA 联合仿真平台** —— 像 CarSim 一样，全部通过图形界面操作 CARLA。

**你的控制算法控制 CarSim 里的车，CarSim 计算车辆动力学，CARLA 负责场景、渲染和传感器。** 每一帧 CarSim 算出的车身位姿、四轮转向角、车轮转速、悬架行程都同步到 CARLA 车辆上，CARLA 自己不算动力学，只照 CarSim 的结果摆放车辆，**转向机构一致**。再配上多视图、传感器套件编辑、数据采集与导出、场景和交通等功能，不写代码也能完成仿真和数据集生产。

```
你的控制算法 (Python) ──油门 / 制动 / 方向盘──▶ CarSim（车辆动力学，经 python_carsim_env 调用）
                                                    │ 位姿 · 车轮 · 悬架（每帧）
                                                    ▼
                          CARLA（场景 · 交通 · 相机 / 激光雷达 / 毫米波雷达 · 画面 · 数据采集）
```

## 🔗 基础：python_carsim_env

[**python_carsim_env**](https://github.com/dyZhou2001/python_carsim_env) 由 **[dyZhou2001](https://github.com/dyZhou2001)** 最初开发并以 MIT 许可证开源，它用 Python（ctypes）直接调用 CarSim 的 VS Solver API（`vs_read_configuration` → `vs_integrate_io` → `vs_terminate_run`），把一次 CarSim 运行封装成 gym 风格的环境：`CarSimEnv.reset()` 读入 `.sim` 并初始化，`CarSimEnv.control_step(action, inner_steps)` 写入导入变量（油门、制动、方向盘）、积分若干步并返回全部导出变量。

本平台和 CarSim 有关的部分都建立在它之上：

| 本平台的功能 | 用到 python_carsim_env 的地方 |
|---|---|
| 联合仿真 | 每一帧调用 `control_step`，CarSim 积分 `帧周期 / t_step` 步，导出变量再同步到 CARLA |
| 你的控制算法 | 算法返回的油门 / 制动 / 方向盘就是 `control_step` 的 action，按 `.sim` 里的导入变量顺序写入 |
| 示例算法 | `controllers/simple_path_follower.py` 直接使用其中的 `SimplePathFollower` |
| 强化学习 | 同一套 `CarSimEnv` 接口（仓库中已有 SAC 示例），可以在训练代码里接上 CARLA 的画面与传感器 |

**感谢 [dyZhou2001](https://github.com/dyZhou2001) 开源 python_carsim_env**：没有这个 Python ⇄ CarSim 的接口，就没有本平台的联合仿真。

版本来源：[dyZhou2001/python_carsim_env](https://github.com/dyZhou2001/python_carsim_env)（原始仓库）→ [yongqianxiao/python_carsim_env](https://github.com/yongqianxiao/python_carsim_env)（fork）→ [Mrchengyuan/python_carsim_env](https://github.com/Mrchengyuan/python_carsim_env)（fork）。本平台用最后这个版本开发和测试；本仓库不包含 python_carsim_env 的代码，只在运行时调用它，使用和再分发请遵守其 MIT 许可证。

使用时把它 clone 到本仓库根目录（界面“CarSim 动力学”页的“python_carsim_env 目录”默认就是 `../python_carsim_env`）：
```bash
git clone https://github.com/Mrchengyuan/python_carsim_env
```
没有 CarSim 许可证时，界面里勾选“模拟 CarSim”，用内置的简化车辆模型代替，先把整条链路跑通。

## 📖 使用文档

| 文档 | 内容 |
|---|---|
| **[Ubuntu 使用指南](docs/Ubuntu使用指南.md)** | 从零安装、原版 / 改版 CARLA、Python 环境、编译界面、桌面一键启动、命令行、测试、常见问题 |
| **[Windows 使用指南](docs/Windows使用指南.md)** | 从零安装、CARLA、Python、界面、一键启动、**接入 CarSim**、改版 CARLA、常见问题 |
| **[界面操作手册](docs/界面操作手册.md)** | 每个页面、每个按钮的说明，数据采集输出格式，常用操作流程 |
| [CarSim 导出变量清单](carsim_carla_bridge/docs/CarSim导出变量清单.md) | CarSim 里要导出哪些变量、单位、坐标约定 |
| [Windows 编译指南](carsim_carla_bridge/docs/Windows编译指南.md) | 在 Windows 上编译改版 CARLA |

## 原版 CARLA 与改版 CARLA

本平台能用两种 CARLA。**两种都能和 CarSim 联合仿真，画面上的同步效果完全一样**，区别只在速度 / IMU 读数和悬架动画。**不确定就先用原版。**

- **原版 CARLA**：官方发布的 [CARLA 0.9.16](https://github.com/carla-simulator/carla/releases/tag/0.9.16)，下载解压就能用。每一帧由后端把车直接摆到 CarSim 算出的位置（界面上叫“兼容模式”）。
- **改版 CARLA**：官方 0.9.16 源码打上本仓库的补丁 [`carla_patches/carla_0.9.16_external_dynamics.patch`](carla_patches/carla_0.9.16_external_dynamics.patch) 后**自己编译**得到。补丁给 CARLA 增加“外部动力学接口”：CarSim 每一帧把完整的车辆状态（位置、姿态、速度、角速度、四轮转向角、车轮转角、悬架行程）一次交给 CARLA，CARLA 里的速度和传感器读数因此是真实值。

| | 原版 CARLA | 改版 CARLA |
|---|---|---|
| 怎么得到 | 从官方下载（约 8 GB） | 自己编译：Ubuntu 约 150 GB 磁盘、2–3 小时；Windows 约 165–200 GB；GitHub 账号要先关联 Epic Games |
| 车身位置、姿态 | ✅ 同步 | ✅ 同步 |
| 四轮转向角、车轮转动 | ✅ 同步 | ✅ 同步 |
| 悬架行程（车轮上下跳动） | ❌ 没有 | ✅ 同步 |
| CARLA 里的 `get_velocity()`、角速度、IMU 传感器 | ❌ 读数为 0 | ✅ CarSim 的真实值 |
| 界面“连接”页显示 | 黄色“原版 CARLA：兼容模式”（或“服务器是原版 CARLA：兼容模式”） | 绿色“改版 CARLA：可用” |
| 默认端口 | 2000 | 3000 |
| Ubuntu 桌面图标 | **CARLA CoSim Studio** | **CARLA CoSim Studio（改版）** |

**什么时候需要改版**：要用 CARLA 自己的速度、IMU 传感器读数（例如采集的数据集里要有 IMU，或别的程序通过 `get_velocity()` 读车速），或者要看到悬架动画。只是调控制算法、看画面、采集相机 / 语义分割 / 深度 / 激光雷达 / 毫米波雷达数据，原版就够了。

### 本仓库里有什么，没有什么

| 有 | 没有 |
|---|---|
| 对 CARLA 的全部改动：`carla_patches/carla_0.9.16_external_dynamics.patch`（外部动力学接口；另外修复了 CARLA 0.9.16 的两个错误：读取卡车、巴士等多轮车辆物理参数时让服务器崩溃的越界；交通车被撞飞或掉出世界时交通管理器死循环、让仿真卡死） | **编译好的改版 CARLA**（也没有原版 CARLA，原版请从官方下载） |
| Linux 编译用的修复补丁：`carla_patches/carla_0.9.16_linux_libpng_url_fix.patch` | |
| 一键编译脚本：`scripts/build_ue4.sh`、`scripts/build_carla.sh` | |
| 启动脚本：`scripts/carla_mod_server.sh`、`scripts/start_studio.sh mod` | |
| 编译教程：[Ubuntu 使用指南 第 8 节](docs/Ubuntu使用指南.md)、[Windows 编译指南](carsim_carla_bridge/docs/Windows编译指南.md) | |

为什么不直接上传编译好的改版 CARLA：一是太大（CARLA 源码加地图资源约 31 GB，它依赖的定制版 UE4 引擎约 93 GB）；二是 UE4 引擎的代码只对关联了 Epic Games 的 GitHub 账号开放，不能再分发。所以每台电脑需要自己编译一次，脚本已经把步骤都写好了。

### 怎么编译改版 CARLA

**Ubuntu 22.04**（本平台就是在这个环境下编译和测试的）。完整步骤和常见问题见 [Ubuntu 使用指南 第 8 节](docs/Ubuntu使用指南.md)，概要：

1. 把 GitHub 账号关联到 Epic Games（免费，只需一次），这样才有权限下载 CARLA 定制版 UE4。
2. `./scripts/build_ue4.sh`：下载并编译 UE4，约 1 小时、95 GB。
3. `./scripts/build_carla.sh`：下载 CARLA 0.9.16 源码到 `carla_src/` → 打上 `carla_patches/` 里的补丁 → 下载地图资源 → 编译 Python 包和 CARLA，约 1 小时。运行前先 `source venv/bin/activate` 并装好 CARLA 的编译依赖（命令见指南 8.3）。
4. 把编译出的改版 Python 包装进单独的环境 `venv_build`：
   ```bash
   python3 -m venv venv_build
   venv_build/bin/pip install carla_src/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl numpy pillow shapely networkx
   venv_build/bin/python -c "import carla; print(hasattr(carla.Vehicle, 'apply_external_state'))"   # 输出 True 就对了
   ```
5. `bash scripts/install_desktop_icons.sh`，双击桌面上的 **CARLA CoSim Studio（改版）**。第一次启动要编译着色器（20–40 分钟），以后约 40 秒（本机实测 42 秒）。
6. 验证：`venv_build/bin/python carsim_carla_bridge/tests/test_modified_carla.py --port 3000`，10 项全部 PASS。

双击图标后 CARLA 没起来或中途消失：看 `~/.cache/carla_cosim_studio/launch_mod.log`（启动过程、CARLA 何时退出）和 `~/.cache/carla_cosim_studio/stop.log`（每次关闭 CARLA 是谁、因为什么），详见 [Ubuntu 使用指南 常见问题](docs/Ubuntu使用指南.md)。启动时关掉“正在启动”进度窗口不影响启动。

**Windows**：按 [Windows 编译指南](carsim_carla_bridge/docs/Windows编译指南.md)（需要 VS 2022；流程和官方 0.9.16 一样，只是编译前多打一个补丁）。Windows 上的编译步骤还没有在实机上完整走过一遍。

### Python 的 carla 包也分原版和改版

后端通过 Python 的 `carla` 包和 CARLA 通信，这个包也有两种：

| 包 | 怎么装 | 能连 |
|---|---|---|
| 原版包 | `pip install carla==0.9.16`（Ubuntu 上装在 `venv`） | 原版和改版 CARLA 都能连，但**都只能用兼容模式** |
| 改版包 | 编译改版 CARLA 时生成的 `.whl`（Ubuntu 上装在 `venv_build`） | 连改版 CARLA 用外部动力学接口；连原版 CARLA **自动改用兼容模式** |

交通管理器（控制背景交通车的 CARLA 模块）运行在 Python 包里，不在服务器里。CARLA 0.9.16 的交通管理器有个错误：一辆交通车被撞飞或掉出世界时它会陷入死循环，整个仿真卡住。**改版包修好了这个错误，连原版 CARLA 时也有效**；用原版包时界面会提示“后端卡住了”并提供“重启后端”按钮。

- 要用改版的功能，**CARLA 服务器和 Python 包都必须是改版**。
- Ubuntu 桌面图标和 `start_studio.sh`：有 `venv_build` 就用它，否则用 `venv`，不用手动切换。
- 界面“CarSim 动力学”页的“CARLA 接口”默认是“自动”，保持默认即可。
- 当前用的是哪种，看界面“连接”页“外部动力学接口”那一行。

## 功能

| 模块 | 能做什么 |
|---|---|
| **CarSim 联合仿真** | CarSim 与 CARLA 同步步进；四轮实际转向角（阿克曼、转向柔度一比一）、车轮转速（可看出打滑 / 抱死）、悬架行程、车身侧倾俯仰全部同步；导出变量可视化编辑与校验 |
| **驾驶模式** | **CarSim 联合仿真**：你的 Python 控制算法（每次运行自动重新加载，改完代码直接再运行）控制 CarSim，CARLA 照 CarSim 结果同步；测试用演示 / 路线跟随 / 键盘驾驶。**CARLA 物理**（不需要 CarSim）：路线跟随、CARLA 自动驾驶（遵守红绿灯、跟车）、键盘驾驶 |
| **传感器套件** | 预设 单前视 / KITTI / nuScenes / 量产车 / 感知真值，按车型尺寸自动布置；俯视图 + 侧视图拖动安装，显示视场角；相机、深度、语义、实例、激光雷达、毫米波雷达、IMU、GNSS |
| **数据采集** | 所有传感器同一帧同步采样；图像 JPG/PNG、点云 .bin/.npy、雷达 CSV；自动生成标定文件（内参 K、外参）、3D 真值框、车辆状态；采集前估算数据量，没有停止条件或空间不足时拒绝开始 |
| **数据浏览与导出** | 逐帧浏览已采集的数据（相机图上叠加 3D 真值框、激光雷达 / 毫米波雷达俯视图、目标列表、播放）；一键导出为 **KITTI** 或 **nuScenes** 格式（nuScenes 可直接用官方工具读取）；删除不需要的数据集 |
| **场景** | 地图切换、天气预设和 9 个参数、背景交通流（车辆 + 行人）、场景对象管理、录制与回放 |
| **视口与监视** | 仿真软件式布局：中间是实时 3D 画面（跟车 / 车头 / 前轮特写 / 俯视 / 任意套件相机），叠加车速、方向盘、踏板仪表和带轨迹的小地图；**多视图**（单画面 / 1+3 / 2×2）同时显示相机、语义分割、深度、实例分割、激光雷达点云和毫米波雷达俯视图；底部面板有实时曲线、车辆状态（位姿、四轮数据）、输出日志 |
| **运行控制** | 运行 / 暂停 / 单步 / 继续 / 停止（F5 / F6 / F10 / Shift+F5）；停止或到时结束后主车停车；运行因到时、出错结束时画面上方提示原因；运行时长默认 0 = 一直运行 |
| **界面** | 菜单栏 + 工具栏（按流程的页签）、左侧工程树、右侧属性面板、底部曲线 / 状态 / 输出，各区域可拖动调整大小；深色 / 浅色主题，中文界面；配置保存为 JSON，命令行和强化学习训练共用 |

| 传感器套件编辑 | 数据采集 |
|---|---|
| ![传感器套件](docs/images/sensor_rig.jpg) | ![数据采集](docs/images/data_collection.jpg) |
| **驾驶模式** | **浅色主题** |
| ![驾驶模式](docs/images/driving_modes.jpg) | ![浅色主题](docs/images/light_theme.jpg) |
| **多视图 2×2：相机 · 语义分割 · 激光雷达 · 深度** | **多视图 1+3：相机 · 毫米波雷达 · 激光雷达 · 深度** |
| ![多视图 2×2](docs/images/multiview_2x2.jpg) | ![多视图 1+3](docs/images/multiview_1p3.jpg) |
| **数据浏览：相机 + 3D 真值框 · 激光雷达俯视 · 目标列表 · 导出** | |
| ![数据浏览](docs/images/dataset_browser.jpg) | |

## 架构

```
┌─ CARLA CoSim Studio ─────────┐  本机 TCP / JSON   ┌─ backend_server.py ──────────────┐   RPC   ┌─ CARLA 0.9.16 ─────────┐
│ C++17 · Dear ImGui · ImPlot  │ ── 命令 ─────────▶ │ carla 客户端                     │ ──────▶ │ 原版：兼容模式          │
│ 界面、传感器套件编辑、曲线    │ ◀─ 实时数据 / 画面 │ CarSim 桥接 · 驾驶 · 数据采集    │         │ 改版：外部动力学接口    │
└──────────────────────────────┘                    └──────────────┬───────────────────┘         └────────────────────────┘
                                                                   │ ctypes (VS API)
                                                           ┌───────▼────────┐
                                                           │ CarSim 求解器   │
                                                           └────────────────┘
```

- 改版 CARLA：`carla_patches/` 给 CARLA 0.9.16 新增 `vehicle.enable_external_dynamics()` / `vehicle.apply_external_state()`，一帧一次下发位姿、速度、角速度、四轮转向 / 转角 / 悬架，`get_velocity()`、IMU 等读数为真实值。
- 原版 CARLA：自动用兼容模式（画面相同，速度类读数为 0，无悬架动画）。两者的区别见 [原版 CARLA 与改版 CARLA](#原版-carla-与改版-carla)。

## 目录

| 目录 | 内容 |
|---|---|
| `cosim_gui/` | 图形界面源码，依赖（ImGui、ImPlot、GLFW、json、Font Awesome）已放在 `third_party/`，编译不需要联网 |
| `carsim_carla_bridge/` | Python 后端、CarSim 桥接、驾驶模式、数据采集、测试和文档 |
| `carla_patches/` | CARLA 0.9.16 补丁（改版 CARLA 就是官方源码打上它编译出来的）：外部动力学接口；以及 Linux 编译用的 libpng 地址修复 |
| `scripts/` | Ubuntu：`build_ue4.sh`、`build_carla.sh`、`carla_server.sh`、`carla_mod_server.sh`、`start_studio.sh`、`stop_carla.sh`、`install_desktop_icons.sh`；`scripts/windows/`：`start_studio.bat`、`stop_carla.bat`、`install_shortcuts.bat`、`env.bat` |
| `docs/` | 截图 |

## 快速开始

### 1. CARLA
- **原版**（先用这个）：下载 [CARLA 0.9.16](https://github.com/carla-simulator/carla/releases/tag/0.9.16)，解压后直接运行。
- **改版**（可选）：本仓库只提供补丁和编译脚本，需要自己编译，见上面的 [原版 CARLA 与改版 CARLA](#原版-carla-与改版-carla)。

### 2. 后端
```bash
pip install numpy pillow shapely networkx
pip install carla==0.9.16          # 原版 carla 包；编译了改版 CARLA 就装改版的 .whl（见上文）
git clone https://github.com/Mrchengyuan/python_carsim_env   # CarSim 的 Python 接口，放在仓库根目录
```

### 3. 界面
**Windows**：从 [Releases](../../releases) 下载 `CARLA_CoSim_Studio_Windows.zip`，解压后运行 `carla_cosim_studio.exe`；或自行编译：
```bat
cd cosim_gui
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```
**Ubuntu 22.04**：
```bash
sudo apt install build-essential cmake libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev fonts-noto-cjk
cd cosim_gui && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
./build/carla_cosim_studio
```

### 4. 启动
- **Ubuntu 一键启动**：运行一次 `bash scripts/install_desktop_icons.sh`，桌面上会出现三个图标：
  - **CARLA CoSim Studio**：原版 CARLA + 界面。启动快、切换地图快，平时先用这个。
  - **CARLA CoSim Studio（改版）**：改版 CARLA + 界面。和 CarSim 联合仿真、需要速度 / IMU 读数和悬架动画完全同步时用；以编辑器模式运行，启动约 40 秒，第一次切换地图很慢，尽量用默认地图。
  - **关闭 CARLA**：关掉所有 CARLA 服务器。两个启动图标不要同时开。
- **Windows**：运行 `scripts\windows\install_shortcuts.bat` 创建桌面快捷方式，或先启动 CARLA 再打开 `carla_cosim_studio.exe`。

### 5. 使用（详细步骤见上面的使用指南）
1. 用桌面图标启动时界面会自动连接 CARLA；直接打开程序时，在 **连接** 页设置 Python 解释器、`carsim_carla_bridge` 目录后点 **连接 CARLA**。
2. **车辆与视角**：选车型和出生点（出生点也是 CarSim 坐标原点），生成主车后中间自动显示实时画面。
3. **CarSim 动力学**：填 `.sim` 文件和导出变量（见 [导出变量清单](carsim_carla_bridge/docs/CarSim导出变量清单.md)）；没有 CarSim 许可证可勾选 **模拟 CarSim** 先跑通链路。
4. **驾驶模式**：选 **CarSim 联合仿真**，在“算法文件 .py”里填你的控制算法（写法见下面）。
5. **传感器套件 / 数据采集** 按需设置，按 **F5** 或点工具栏 **运行**；点 **停止** 结束，主车会停住。

### 6. 接入你的控制算法
`.sim` 里的油门、制动、方向盘导入变量保持 **REPLACE**（和 python_carsim_env 一样）。写一个 Python 文件，例如 `carsim_carla_bridge/controllers/my_controller.py`：
```python
class Controller:
    def reset(self):                          # 可选，每次运行开始调用一次
        self.integral = 0.0

    def control(self, exports, t, dt):        # 每帧调用一次
        vx = exports["Vx"]                    # 按变量名取 CarSim 导出变量（CarSim 单位）
        ...                                   # 你的算法
        return [throttle, brake, steer_sw]    # 按 .sim 里导入变量的顺序
```
- `exports`：全部 CarSim 导出变量，键是导出变量名；`t`：CarSim 时间；`dt`：控制周期（= 仿真步长）。
- 界面里的相对路径以 `carsim_carla_bridge` 目录为准（命令行里以当前目录为准）；每次点“运行”都会重新加载这个文件，改完代码直接再运行，不用重启界面。
- 示例：`controllers/example_controller.py`（定速 + 蛇形）、`controllers/simple_path_follower.py`（python_carsim_env 里的 SimplePathFollower）。

命令行和强化学习训练用同一份配置：
```bash
python carsim_carla_bridge/run_cosim.py --config cosim_config.json                      # 界面保存的配置
python carsim_carla_bridge/run_cosim.py --sim simfile.sim --controller carsim_carla_bridge/controllers/my_controller.py
python carsim_carla_bridge/run_cosim.py --mock --duration 20                             # 不需要 CarSim
```

## 坐标与同步

- CarSim（ISO 8855：x 前 y 左 z 上）→ CARLA（x 前 y 右 z 上）：`y → -y`，`yaw → -yaw`，`pitch → -pitch`，`roll` 不变，车轮转角取反；已在 CARLA 0.9.16 上用旋转矩阵和车轮骨骼实测验证。
- CARLA 同步模式，每帧 CarSim 积分 `帧周期 / t_step` 步；`apply_external_state` 为阻塞调用，保证状态落在同一帧（300 帧 0 延迟，约 0.65 ms/帧）。

## 测试

在 Ubuntu 22.04 上，**原版 CARLA 0.9.16 和改版 CARLA 各跑一遍**（除 `test_modified_carla.py` 只对改版），两种 Python carla 包也都跑过：

| 测试 | 内容 | 结果 |
|---|---|---|
| `tests/test_coords.py` | 坐标换算与 CARLA 旋转矩阵对照 | 5/5 |
| `tests/test_backend.py` | 界面后端全部命令（地图、天气、交通、传感器、多视图、录制、联合仿真、暂停 / 单步、出生点被占时启动失败不丢主车） | 35/35 |
| `tests/test_features.py` | 传感器套件、磁盘保护、各驾驶模式、自定义控制算法、停止后停车、3 帧多传感器采集 | 18/18 |
| `tests/test_dataset.py` | 小规模采集 → 浏览渲染 → KITTI / nuScenes 导出；用语义激光雷达验证坐标约定，用 KITTI 文件本身复算框内点数，装了 nuscenes-devkit 时用官方工具交叉验证；缺帧时 KITTI 编号连续、导出中拒绝删除、路径含 `[ ]` 等特殊字符 | 22/22 |
| `tests/test_robustness.py` | 后端抗异常：控制算法在导入或运行时调用 `sys.exit`、输出 NaN，格式错误的请求，不存在的车型（主车和传感器保留），视图建不起来时通知界面，改版客户端连原版服务器时自动用兼容模式，测量全部车型（含 6 轮卡车），主车被 CARLA 删除（开出地图掉出世界）时运行带原因结束、界面得知，交通车被撞飞时运行不卡死（改版包），重新连接时清理主车 / 交通 / 视图，非有限数值安全发送 | 原版包 21/21，改版包 23/23；改版 CARLA 上 22/22 |
| `tests/test_all_vehicles.py` | 41 种车型（含自行车、摩托车、6 轮卡车、巴士）逐一当主车做联合仿真，CARLA 服务器不能崩 | 41/41 |
| `tests/test_modified_carla.py` | 改版 CARLA：位姿、速度、角速度、IMU、四轮转向、悬架、物理交接 | 10/10 |
| 界面 `--tour` | 自动操作全部页面并截图；用**真实鼠标点击**测试运行 / 暂停 / 单步 / 继续 / 停止、视口按钮、多视图布局与视图内容切换、页签和工程树、数据浏览（打开 / 逐帧 / 播放）与导出 | 42/42 |

## 已知限制

- 真实 CarSim 的导出变量名和单位需按你的 `.sim` 核对（在 Windows 上运行 CarSim）。
- 车辆由外部动力学驱动时为运动学刚体，不产生碰撞响应；CARLA 地图路面高低起伏而 CarSim 用平路时，把“高度模式”设为“贴合 CARLA 路面”。
- “路线跟随”只做路径和车速跟踪，不看红绿灯；要遵守红绿灯请用“CARLA 自动驾驶”（仅 CARLA 物理）。
- Windows 版界面和 `.bat` 脚本由 Linux 交叉编译 / 编写，尚未在 Windows 实机上完整运行过。
- 用原版 Python carla 包时，交通车被撞飞仍可能让后端卡住（CARLA 0.9.16 自身的错误，见上文）：界面会提示，点“重启后端”即可继续。Ubuntu 桌面图标默认用改版包，没有这个问题。
- 改版 CARLA 以编辑器模式运行时首次切换地图较慢；打包版（`make package`）无此问题。

## 致谢

[python_carsim_env](https://github.com/dyZhou2001/python_carsim_env)（CarSim 接口，原作者 [dyZhou2001](https://github.com/dyZhou2001)）· [CARLA](https://github.com/carla-simulator/carla) · [Dear ImGui](https://github.com/ocornut/imgui) · [ImPlot](https://github.com/epezent/implot) · [GLFW](https://github.com/glfw/glfw) · [nlohmann/json](https://github.com/nlohmann/json) · [Font Awesome](https://fontawesome.com)
