# CarSim 导出变量清单（桥接到 CARLA 所需）

在 CarSim 的 Import/Export 界面（或 `.sim` 对应的 I/O Channels: Export 数据集）里按下表添加 Export 变量，
然后把 **同样的顺序** 写进 `config.py` 的 `EXPORT_NAMES`。桥接代码按名字查找变量，
所以你原来已有的导出变量（横向误差、车速等）可以保留在任意位置，只要 `EXPORT_NAMES` 顺序与 `.sim` 一致。

> 变量名以 CarSim 标准命名为准。不同 CarSim 版本/车型（尤其是后轴转向、多轴车）个别名字可能不同，
> 添加时请在 CarSim 的 Export 变量选择列表里核对拼写。

## 1. 必需变量（车身位姿 + 前轮转角）

| 变量 | 含义 | 单位（CarSim 用户单位） | 用途 |
|---|---|---|---|
| `Xo` | 簧载质量坐标原点 全局 X | m | 车辆位置 |
| `Yo` | 簧载质量坐标原点 全局 Y | m | 车辆位置（ISO：Y 向左） |
| `Zo` | 簧载质量坐标原点 全局 Z | m | 车辆高度 |
| `Yaw` | 横摆角 | deg | 航向（ISO：逆时针为正） |
| `Pitch` | 俯仰角 | deg | 车身俯仰（ISO：车头向下为正） |
| `Roll` | 侧倾角 | deg | 车身侧倾（ISO：右侧下沉为正） |
| `Steer_L1` | 左前轮转向角（车轮实际转角） | deg | CARLA 左前轮转向 |
| `Steer_R1` | 右前轮转向角 | deg | CARLA 右前轮转向 |

**转向机构一致性的关键**：CARLA 两个前轮直接用 CarSim 算出的 `Steer_L1` / `Steer_R1`，
而不是用方向盘角除以一个固定传动比。因此 CarSim 模型里的阿克曼几何、转向系统柔度、
轮胎回正力矩引起的转角变化都会一比一反映到 CARLA 车轮上（内外轮转角不同）。

## 2. 推荐变量（同步质量更高）

| 变量 | 含义 | 单位 | 不导出时桥接的退化做法 |
|---|---|---|---|
| `Vx` | 车身纵向速度（车体坐标） | km/h | 用相邻两帧位置差分 |
| `Vy` | 车身侧向速度（车体坐标） | km/h | 同上 |
| `AVx` | 侧倾角速度（车体坐标） | deg/s | 用相邻两帧姿态差分 |
| `AVy` | 俯仰角速度（车体坐标） | deg/s | 同上 |
| `AVz` | 横摆角速度（车体坐标） | deg/s | 同上 |
| `AVy_L1` `AVy_R1` `AVy_L2` `AVy_R2` | 四个车轮自转角速度 | rpm | 用车速/轮胎半径（看不出打滑、抱死） |
| `Steer_L2` `Steer_R2` | 后轮转向角（四轮转向车） | deg | 后轮转角为 0 |
| `Steer_SW` | 方向盘转角 | deg | `get_control().steer` 报 0 |
| `Throttle` | 油门开度 | 0–1 | `get_control().throttle` 报 0 |
| `GearStat` | 当前挡位 | – | `get_control().gear` 报 0 |

有了 `Vx Vy AVx AVy AVz`，改版 CARLA 里 `vehicle.get_velocity()`、`get_angular_velocity()`、
IMU 陀螺仪读数就是 CarSim 的真实值（原版 CARLA 里物理关闭后这些读数都是 0）。
有了 `AVy_*`，车轮转速来自 CarSim 的轮胎模型，急加速打滑、制动抱死在画面上都能看出来。

## 3. 悬架行程（车轮相对车身上下跳动）

| 变量 | 含义 | 单位 | 不导出时 |
|---|---|---|---|
| `Jnc_L1` `Jnc_R1` `Jnc_L2` `Jnc_R2` | 四轮悬架动行程（压缩为正） | mm | 车轮固定在设计位置 |

车身的侧倾、俯仰、升沉通过 `Roll/Pitch/Zo` 同步，车轮相对车身的跳动通过 `Jnc_*` 同步。
这样过减速带、转弯外侧压缩、制动点头时，车轮和车身的相对位置都和 CarSim 一致。
这个功能只在改版 CARLA 上有效，原版 CARLA 没有设置车轮高度的 Python 接口。
（CARLA 定制版 UE4 本身就有 `VehicleAnimInstance::SetWheelHeight`，不需要改引擎。）

## 4. 单位与坐标约定

- 桥接默认 CarSim 导出的是**用户单位**（deg、km/h、deg/s、rpm）。你的项目里车速和 50 km/h 的目标车速比较、
  转向输入按度计算，也说明导出的是用户单位。如果你的 run 导出的是 SI 内部单位，改 `config.py` 里的 `UNITS`。
- CarSim 使用 ISO 8855 坐标系（X 前、Y 左、Z 上），CARLA/UE4 是 X 前、Y 右、Z 上。
  换算规则：`y → -y`，`yaw → -yaw`，`pitch → -pitch`，`roll` 不变，车轮转角取反。
  这些都已写在 `coords.py` 里，并在 CARLA 0.9.16 上用实测数据验证过。
- **参考点**：CarSim 的 `Xo/Yo/Zo` 是簧载质量坐标原点，桥接默认它位于**前轴中心、地面高度**，
  会自动用 CARLA 车型的前轴位置换算到 CARLA 车辆原点。验证方法：静止时 `Zo` 应接近 0。
  如果你的模型不是这样，在 `config.py` 里把 `CARSIM_REFERENCE_POINT` 改成实际偏移。

## 5. 坐标原点对齐

CarSim 全局原点 (0,0,0)、yaw=0 会放到 CARLA 的一个 spawn point 上（`--spawn-index` 选择）。
如果 CarSim 用的是自己的道路，而 CARLA 地图道路不同，车辆会"开出路面"，这只是地图不一致，
不是同步问题。解决办法有两种：
- 在 CarSim 里导入和 CARLA 地图一致的道路：CARLA 地图有 OpenDRIVE 文件，路网中心线可以导出后作为 CarSim 的路径和路面。
- 把 `config.py` 的 `Z_MODE` 设为 `"ground"`，高度贴合 CARLA 路面，平面运动仍用 CarSim 的结果。
