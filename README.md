# CARLA CoSim Studio

CarSim ⇄ CARLA 联合仿真平台：像 CarSim 一样全部通过图形界面操作 CARLA。

- **CarSim 动力学驱动 CARLA 车辆**：车身位姿、四轮转向角、车轮转速、悬架行程逐帧同步（转向机构一致）
- **图形界面**（C++ / Dear ImGui）：地图天气、交通流、车辆、传感器套件编辑器、驾驶模式、数据采集、录制回放、实时画面
- **数据采集**：多传感器同步采集图像 / 点云 / 雷达，自动输出标定文件和 3D 真值标注，带磁盘空间保护
- **CARLA 补丁**：新增外部动力学接口 `vehicle.apply_external_state()`，速度 / IMU 读数为真实值

## 目录

| 目录 | 内容 |
|---|---|
| `cosim_gui/` | 图形界面源码（依赖已放在 `third_party/`，编译不需要联网） |
| `carsim_carla_bridge/` | Python 后端、CarSim 桥接、数据采集、测试和文档 |
| `carla_patches/` | 对 CARLA 0.9.16 的修改（外部动力学接口）；Linux 编译另需 libpng 地址修复 |
| `scripts/` | 编译 UE4 / CARLA、启动服务器、桌面一键启动脚本 |

CarSim 的 Python 接口来自 [python_carsim_env](https://github.com/Mrchengyuan/python_carsim_env)，需要单独克隆到本仓库根目录。

## 快速开始

1. **CARLA**：用原版 CARLA 0.9.16 即可运行（兼容模式）；要用外部动力学接口，按
   [`carsim_carla_bridge/docs/Windows编译指南.md`](carsim_carla_bridge/docs/Windows编译指南.md)
   给 CARLA 0.9.16 源码打上 `carla_patches/carla_0.9.16_external_dynamics.patch` 后编译。
2. **界面**：
   ```bash
   cd cosim_gui
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release     # Windows: -G "Visual Studio 17 2022" -A x64
   cmake --build build --config Release
   ```
3. **后端**：`pip install carla==0.9.16 numpy pillow shapely networkx`（或用补丁编译出的 carla wheel），
   启动界面后在“连接”页设置 Python 解释器和 `carsim_carla_bridge` 目录。

命令行 / 强化学习训练同样可用：`python carsim_carla_bridge/run_cosim.py --config cosim_config.json`

## 文档

- [CarSim 导出变量清单](carsim_carla_bridge/docs/CarSim导出变量清单.md)
- [Windows 编译指南](carsim_carla_bridge/docs/Windows编译指南.md)
- [界面说明](cosim_gui/README.md) · [桥接说明](carsim_carla_bridge/README.md)
