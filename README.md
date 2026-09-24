# CARLA CoSim Studio

**CarSim ⇄ CARLA 联合仿真平台** —— 像 CarSim 一样，全部通过图形界面操作 CARLA。

CarSim 负责车辆动力学，CARLA 负责场景、渲染和传感器。每一帧 CarSim 算出的车身位姿、四轮转向角、车轮转速、悬架行程都同步到 CARLA 车辆上，**转向机构一致**；再配上传感器套件编辑、驾驶模式、数据采集等功能，不写代码也能完成仿真和数据集生产。

![运行中](docs/images/running.jpg)

## 功能

| 模块 | 能做什么 |
|---|---|
| **CarSim 联合仿真** | CarSim 与 CARLA 同步步进；四轮实际转向角（阿克曼、转向柔度一比一）、车轮转速（可看出打滑 / 抱死）、悬架行程、车身侧倾俯仰全部同步；导出变量可视化编辑与校验 |
| **驾驶模式** | CarSim 动力学或 CARLA 自带物理；路线跟随（路网规划 + 纯跟踪）、CARLA 自动驾驶、键盘驾驶（WASD）、演示、PID 路径跟踪 |
| **传感器套件** | 预设 单前视 / KITTI / nuScenes / 量产车 / 感知真值，按车型尺寸自动布置；俯视图 + 侧视图拖动安装，显示视场角；相机、深度、语义、实例、激光雷达、毫米波雷达、IMU、GNSS |
| **数据采集** | 所有传感器同一帧同步采样；图像 JPG/PNG、点云 .bin/.npy、雷达 CSV；自动生成标定文件（内参 K、外参）、3D 真值框、车辆状态；采集前估算数据量，没有停止条件或空间不足时拒绝开始 |
| **场景** | 地图切换、天气预设和 9 个参数、背景交通流（车辆 + 行人）、场景对象管理、录制与回放 |
| **监视** | 界面内实时画面（跟车 / 车头 / 前轮特写 / 俯视 / 任意套件相机）、车速与控制量、四轮数据表、实时曲线 |
| **界面** | 深色 / 浅色主题，中文界面；配置保存为 JSON，命令行和强化学习训练共用 |

| 传感器套件编辑 | 数据采集 |
|---|---|
| ![传感器套件](docs/images/sensor_rig.jpg) | ![数据采集](docs/images/data_collection.jpg) |
| **驾驶模式** | **浅色主题** |
| ![驾驶模式](docs/images/driving_modes.jpg) | ![浅色主题](docs/images/light_theme.jpg) |

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

- `carla_patches/` 给 CARLA 0.9.16 新增 `vehicle.enable_external_dynamics()` / `vehicle.apply_external_state()`：一帧一次下发位姿、速度、角速度、四轮转向 / 转角 / 悬架，`get_velocity()`、IMU 等读数为真实值。
- 不打补丁也能用：自动退化为原版接口（画面相同，速度类读数为 0，无悬架动画）。

## 目录

| 目录 | 内容 |
|---|---|
| `cosim_gui/` | 图形界面源码，依赖（ImGui、ImPlot、GLFW、json、Font Awesome）已放在 `third_party/`，编译不需要联网 |
| `carsim_carla_bridge/` | Python 后端、CarSim 桥接、驾驶模式、数据采集、测试和文档 |
| `carla_patches/` | CARLA 0.9.16 补丁：外部动力学接口；以及 Linux 编译用的 libpng 地址修复 |
| `scripts/` | 编译 UE4 / CARLA、启动服务器、Ubuntu 桌面一键启动脚本 |
| `docs/` | 截图 |

## 快速开始

### 1. CARLA
- **原版**：下载 [CARLA 0.9.16](https://github.com/carla-simulator/carla/releases/tag/0.9.16) 直接运行即可。
- **改版**（推荐）：在 0.9.16 源码上 `git apply carla_patches/carla_0.9.16_external_dynamics.patch` 后编译，
  步骤见 [Windows 编译指南](carsim_carla_bridge/docs/Windows编译指南.md)（Linux 流程相同，另打 libpng 补丁）。

### 2. 后端
```bash
pip install numpy pillow shapely networkx
pip install carla==0.9.16          # 或安装改版编译出的 PythonAPI/carla/dist/*.whl
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

### 4. 使用
1. 启动 CARLA，打开界面，在 **连接** 页设置 Python 解释器、`carsim_carla_bridge` 目录，点 **连接 CARLA**。
2. **车辆与视角**：选车型和出生点（出生点也是 CarSim 坐标原点）。
3. **CarSim 动力学**：填 `.sim` 文件和导出变量（见 [导出变量清单](carsim_carla_bridge/docs/CarSim导出变量清单.md)）；没有 CarSim 许可证可勾选 **模拟 CarSim** 先跑通链路。
4. **驾驶模式 / 传感器套件 / 数据采集** 按需设置，点顶部 **运行**。

命令行和强化学习训练用同一份配置：
```bash
python carsim_carla_bridge/run_cosim.py --config cosim_config.json
python carsim_carla_bridge/run_cosim.py --mock --duration 20       # 不需要 CarSim
```

## 坐标与同步

- CarSim（ISO 8855：x 前 y 左 z 上）→ CARLA（x 前 y 右 z 上）：`y → -y`，`yaw → -yaw`，`pitch → -pitch`，`roll` 不变，车轮转角取反；已在 CARLA 0.9.16 上用旋转矩阵和车轮骨骼实测验证。
- CARLA 同步模式，每帧 CarSim 积分 `帧周期 / t_step` 步；`apply_external_state` 为阻塞调用，保证状态落在同一帧（300 帧 0 延迟，约 0.65 ms/帧）。

## 测试

在 Ubuntu 22.04 + CARLA 0.9.16 上：

| 测试 | 内容 | 结果 |
|---|---|---|
| `tests/test_coords.py` | 坐标换算与 CARLA 旋转矩阵对照 | 5/5 |
| `tests/test_backend.py` | 界面后端全部命令（地图、天气、交通、传感器、录制、联合仿真） | 30/30 |
| `tests/test_features.py` | 传感器套件、磁盘保护、四种驾驶模式、3 帧多传感器采集 | 16/16 |
| `tests/test_modified_carla.py` | 改版 CARLA：位姿、速度、角速度、IMU、四轮转向、悬架、物理交接 | 11/11 |
| 界面 `--tour` | 自动操作全部页面并截图 | 23/23 |

## 已知限制

- 真实 CarSim 的导出变量名和单位需按你的 `.sim` 核对（在 Windows 上运行 CarSim）。
- 车辆由外部动力学驱动时为运动学刚体，不产生碰撞响应。
- 改版 CARLA 以编辑器模式运行时首次切换地图较慢；打包版（`make package`）无此问题。

## 致谢

[CARLA](https://github.com/carla-simulator/carla) · [Dear ImGui](https://github.com/ocornut/imgui) · [ImPlot](https://github.com/epezent/implot) · [GLFW](https://github.com/glfw/glfw) · [nlohmann/json](https://github.com/nlohmann/json) · [Font Awesome](https://fontawesome.com) · [python_carsim_env](https://github.com/Mrchengyuan/python_carsim_env)
