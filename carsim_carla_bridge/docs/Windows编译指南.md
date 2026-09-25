# 在 Windows 上编译改版 CARLA 0.9.16（CarSim 外部动力学接口）

本补丁只改 CARLA 的 Unreal 插件、LibCarla 和 PythonAPI，**不改 UE4 引擎**
（悬架行程用的是 CARLA 定制版 UE4 已有的 `VehicleAnimInstance::SetWheelHeight`），
所以编译流程和官方 0.9.16 完全一样，只是在编译前多打一个补丁。
官方文档（与源码里的 `Docs/build_windows.md` 一致）：<https://carla.readthedocs.io/en/0.9.16/build_windows/>

## 0. 需要准备

| 项目 | 要求 |
|---|---|
| 系统 | Windows 10/11 x64 |
| 磁盘 | ≥ 165 GB（UE4 约 133 GB，CARLA 约 32 GB），建议 SSD |
| 显卡 | ≥ 6 GB 显存 |
| Visual Studio | **2022**（官方 0.9.16 推荐；装 "使用 C++ 的桌面开发" + Windows 10/11 SDK + .NET 4.8 SDK） |
| CMake | ≥ 3.15 |
| Make | **GnuWin32 Make 3.81**（其他版本可能编译失败） |
| 7-Zip | 用于自动解压资源包 |
| Python | 3.x **64 位**，`pip >= 20.3`，并安装 `setuptools` `wheel` |
| Git | 任意新版本 |
| GitHub 账号 | **必须关联 Epic Games 账号**，否则无权克隆 UE4 源码 |

## 1. 编译 CARLA 定制版 UE4（只需一次，最耗时）

1. 在 <https://www.unrealengine.com/> 登录 Epic 账号 → 个人设置 → Connections → 关联 GitHub，
   然后到 GitHub 接受 EpicGames 组织邀请。
2. 在一个**路径很短**的位置（例如 `C:\UE4`；路径太长 Setup.bat 会报错）：
   ```bat
   git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git C:\UE4
   cd C:\UE4
   Setup.bat
   GenerateProjectFiles.bat
   ```
3. 用 VS 2022 打开 `C:\UE4\UE4.sln`，配置选 **Development Editor / Win64**，右键 `UE4` 项目 → Build。
   视机器配置需要 1~3 小时。
4. 新建系统环境变量 `UE4_ROOT = C:\UE4`。

## 2. 获取 CARLA 0.9.16 源码并打补丁

```bat
git clone --depth 1 -b 0.9.16 https://github.com/carla-simulator/carla.git C:\carla
cd C:\carla
git apply --check carla_0.9.16_external_dynamics.patch
git apply carla_0.9.16_external_dynamics.patch
git status
```
`git status` 应显示 14 个修改文件和 3 个新文件：
```
LibCarla/source/carla/rpc/VehicleExternalState.h                                   (新)
Unreal/.../Vehicle/MovementComponents/ExternalDynamicsMovementComponent.h/.cpp     (新)
```
补丁文件在本仓库的 `carla_patches/carla_0.9.16_external_dynamics.patch`。

> 注意：一定要用 **0.9.16 标签**，不要用 `ue4-dev` 分支。补丁基于 0.9.16，
> 而且 `Update.bat` 会按这个版本下载对应的资源包。

## 3. 下载资源并编译

以下命令都在 **"x64 Native Tools Command Prompt for VS 2022"** 里执行：
```bat
cd C:\carla
Update.bat            :: 下载地图/车辆等资源，约 20 GB
make PythonAPI        :: 编译 LibCarla 客户端 + Python 包（wheel 在 PythonAPI\carla\dist）
make launch           :: 编译 Unreal 插件并打开 UE4 编辑器，点 Play 即启动服务器
```
可选：`make package` 生成和官方发布包一样的独立可执行版本，之后不用每次开编辑器。

## 4. 验证补丁生效

1. 安装刚编译的 Python 包（`make PythonAPI` 通常已自动装好）：
   ```bat
   pip install --force-reinstall PythonAPI\carla\dist\carla-0.9.16-cp3xx-cp3xx-win_amd64.whl
   python -c "import carla; print(hasattr(carla.Vehicle, 'apply_external_state'))"
   ```
   应输出 `True`。
2. 编辑器里点 Play 后运行桥接的模拟测试（不需要 CarSim）：
   ```bat
   cd carsim_carla_bridge
   python run_cosim.py --mock --duration 20
   ```
   第一行应打印 `external-dynamics API: yes (modified CARLA)`。
3. 接真实 CarSim：
   ```bat
   python run_cosim.py --sim C:\path\to\simfile.sim --carsim-repo ..\python_carsim_env
   ```

## 5. 补丁改了什么

| 层 | 文件 | 改动 |
|---|---|---|
| 服务器 / UE4 插件 | `ExternalDynamicsMovementComponent.h/.cpp`（新） | 新的车辆运动组件：关闭 PhysX 车辆物理，车身改为运动学刚体，按外部状态设置位姿、车轮转向、转角和悬架行程 |
| | `CarlaActor.h/.cpp` | `EnableExternalDynamics` / `ApplyVehicleExternalState`；`GetActorAngularVelocity` 返回外部角速度 |
| | `CarlaServer.cpp` | 新 RPC：`enable_external_dynamics`、`apply_vehicle_external_state` |
| | `WorldObserver.cpp`、`InertialMeasurementUnit.cpp` | 快照和 IMU 使用外部角速度（运动学刚体的 PhysX 角速度恒为 0） |
| | `CarlaServerResponse.h/.cpp` | 新错误码 `ExternalDynamicsNotEnabled` |
| 公共 | `rpc/VehicleExternalState.h`（新） | RPC 数据结构：位姿、线速度、角速度、各轮转向角、各轮转角、各轮悬架行程、驾驶输入 |
| 客户端 | `Client`、`Simulator`、`Vehicle` | C++ 客户端接口 |
| Python | `Actor.cpp`、`libcarla.pyi` | `vehicle.enable_external_dynamics()`、`vehicle.apply_external_state(...)` |

与原版 CARLA 相比，改版的好处：
- **一帧只发一个 RPC，并且在同一帧内原子生效**：原版需要调用 `set_transform`，再对每个车轮分别调用 `set_wheel_steer_direction` 和 `set_wheel_pitch_angle`，每帧共 9 次调用。
- **速度类读数是真实值**：`get_velocity()`、`get_angular_velocity()`、`get_control()`、IMU、加速度都返回 CarSim 的数据。原版关闭物理后这些读数都是 0，依赖它们的传感器、Traffic Manager 和 ROS 桥都会出错。
- **可以随时切回 CARLA 物理**：调用 `vehicle.restore_physx_physics()` 即可，切换时带着当前速度交接。

## 6. 编译验证情况（已在 Ubuntu 22.04 上完整编译并运行）

- UE4 定制版、CARLA 编辑器（含本补丁的 Unreal 插件）、LibCarla 和 PythonAPI 都已在服务器上用官方工具链编译通过，0 个错误。
- 改版服务器上跑 `tests/test_modified_carla.py`，10 项全部通过：位姿零误差、`get_velocity` 和角速度是真实值、
  IMU 陀螺仪读到正确的横摆角速度、四轮转向、悬架（车轮骨骼按指令移动 4 cm）、切回 PhysX 时速度交接。
  连续 300 帧 0 帧延迟，`apply_external_state` 每帧约 0.65 ms。
- `apply_external_state` 是**阻塞调用**：CARLA 服务器用多线程处理 RPC，如果不等待返回，偶尔会被下一次 tick 抢先，
  导致那一帧显示上一帧的位姿（实测 40 帧里有 1～3 帧），所以改成等服务器确认后再返回。
- 编辑器直接以 `-game` 模式运行时，UE4 会实时生成网格距离场，渲染线程可能读到生成一半的数据而崩溃。
  服务器上的启动脚本加了 `r.GenerateMeshDistanceFields=False` 来规避。打包版（`make package`）没有这个问题；
  编辑器模式下首次切换地图也会因为现场编译着色器而很慢，打包版约 6 秒。
- Linux 专用：CARLA 0.9.16 的 `Setup.sh` 里 libpng 下载地址已失效，修复放在单独的
  `carla_0.9.16_linux_libpng_url_fix.patch` 里，Windows 不需要。

## 7. 旧版说明

- **客户端部分已验证**：在服务器上用 g++ 12 对 LibCarla 客户端（`Vehicle.cpp`、`Client.cpp`）和整个
  PythonAPI 绑定（`libcarla.cpp`）做了编译检查，结果为 0 错误。RPC 结构体的 msgpack 序列化往返测试也已通过。
- **Unreal 插件部分未编译**：这部分需要 UE4 引擎，服务器上没有。代码按 CARLA 现有的
  CarSim/Chrono 组件写法编写，调用的都是 0.9.16 插件里已经在用的接口，
  但**第一次 `make launch` 仍可能遇到 MSVC 编译报错**。遇到报错把输出发给我即可。
