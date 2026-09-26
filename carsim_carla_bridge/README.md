# CarSim ⇄ CARLA 联合仿真桥接

CarSim 负责全部车辆动力学计算，CARLA 负责场景、渲染和传感器。每一帧的流程：

```
控制器 ──[油门, 制动, 方向盘角]──▶ CarSim（python_carsim_env，vs_integrate_io × N 步）
                                         │ 导出变量：Xo..Roll, Steer_L1/R1..., AVy_L1...
                                         ▼
                                 bridge.py 坐标换算（ISO → UE）
                                         │ 位姿 + 速度 + 角速度 + 各轮转向角 + 各轮转角
                                         ▼
               CARLA vehicle.apply_external_state(...)   ← 改版 CARLA，一次 RPC
               （原版 CARLA 自动退化为 set_transform + 逐轮动画）
                                         ▼
                                   world.tick()  同步模式，两边共用同一个时钟
```

## 文件

| 文件 | 作用 |
|---|---|
| `run_cosim.py` | 主程序：同步模式主循环、日志、可选录像 |
| `bridge.py` | `CarlaVehicleSync`：把 CarSim 导出向量转换成 CARLA 状态并下发 |
| `coords.py` | ISO 8855 ↔ UE 坐标 / 欧拉角 / 角速度换算 |
| `config.py` | **需要按你的 .sim 修改**：导出变量顺序、单位、参考点 |
| `mock_carsim.py` | 没有 CarSim 时用的替身（运动学自行车模型），接口与 `CarSimEnv` 相同 |
| `tests/test_coords.py` | 坐标换算单元测试（与 `carla.Transform.get_matrix()` 对照） |
| `tests/check_wheels.py` | 用车轮骨骼姿态实测转向 / 转角的正负号 |
| `tests/snapshot_steer.py` | 大转角近景截图，肉眼确认转向机构效果 |
| `docs/CarSim导出变量清单.md` | CarSim 里要导出哪些变量 |
| `docs/Windows编译指南.md` | 改版 CARLA 在 Windows 上的编译步骤 |

## 使用

```bash
# 1) 启动 CARLA（改版或原版都可以），然后：
python run_cosim.py --mock --duration 20 --record out/      # 不需要 CarSim，先验证链路
# 2) 接入真实 CarSim（Windows）
python run_cosim.py --sim C:\CarSim\simfile.sim --carsim-repo ..\python_carsim_env
python run_cosim.py --sim C:\CarSim\simfile.sim --controller controllers\my_controller.py # 你的控制算法（写法见 controllers\example_controller.py）
```
常用参数：`--frame-dt 0.02`（CARLA 帧周期，最好是 CarSim `t_step` 的整数倍）、
`--spawn-index`（把 CarSim 原点放在哪个 spawn point）、`--vehicle`（CARLA 车型）。
`--no-external-api` 强制使用原版 CARLA 的退化模式。

在你自己的 RL 训练代码里使用，只需要在每次 `env.control_step()` 之后加两行：
```python
sync = CarlaVehicleSync(world, vehicle, anchor_transform)   # 初始化一次
...
obs, r, done, info = env.control_step(action, inner_steps)
sync.sync(obs, env.t_current, frame_dt)
world.tick()
```

## 转向机构一致性怎么保证

- **前轮转角**：直接使用 CarSim 输出的每个车轮的实际转向角（`Steer_L1`/`Steer_R1`），CARLA 不自己算。
  阿克曼内外轮转角差、转向柔度、回正力矩引起的转角变化都一比一同步。
  在 CARLA 0.9.16 上用骨骼姿态测过：下发 20°，车轮就转 20°。
- **后轮转向**：导出 `Steer_L2`/`Steer_R2` 就会自动同步。
- **车轮转速**：对 CarSim 的轮速 `AVy_*` 做积分，打滑和抱死都能在画面上看到。
- **车身姿态**：侧倾、俯仰、升沉直接来自 CarSim，不经过 CARLA 物理引擎。
- **悬架行程**：导出 `Jnc_*` 后，每个车轮相对车身的上下跳动同步（仅改版 CARLA）。

## 已验证 / 未验证

在服务器上运行原版 CARLA 0.9.16，接模拟 CarSim，已经验证：
- 1000 帧里 CARLA 渲染的位姿与下发指令的误差为 0 m / 0.0001°。
- 坐标换算和 CARLA 自己的旋转矩阵在 500 组随机姿态下完全一致。
- 车轮转向角和转角的正负号用骨骼姿态实测确认。
- 速度：50 Hz 帧率 2.1 倍实时，20 Hz 帧率 5.2 倍实时（不录像，原版退化模式）。

**还没有验证的部分**：
- 真实 CarSim 的导出变量名和单位（CarSim 只能在你的 Windows 上跑）。
- 真实 CarSim 输出下参考点位置是否准确。
- 改版 CARLA 的 Unreal 插件已在 Ubuntu 上编译并测试（悬架方向、四轮转向、IMU 等，见 `tests/test_modified_carla.py`），Windows 上尚未编译。

## 已知限制

- 车辆被设为运动学刚体、按瞬移方式更新位姿，所以不会产生碰撞响应（不会被撞开）。需要碰撞时，可以在 CarSim 端处理，
  或者调用 `restore_physx_physics()` 切回 CARLA 物理。
- 目前一次只同步一辆 CarSim 车。多辆车需要多开 CarSim 进程，并给每辆车建一个 `CarlaVehicleSync`。
