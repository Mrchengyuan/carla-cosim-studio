# Windows 使用指南

从一台 Windows 10 / 11 电脑开始，到双击桌面图标就能用 CARLA CoSim Studio，并接入真实 CarSim。逐步说明。
界面每个页面怎么操作见 [界面操作手册](界面操作手册.md)。

> **测试说明**：界面程序（`carla_cosim_studio.exe`）和 `scripts\windows\*.bat` 是在 Ubuntu 上交叉编译 / 编写的，
> Windows 专用代码已经通过编译检查，但**还没有在 Windows 实机上运行过**。遇到问题请把 `carsim_carla_bridge\backend.log`
> 和界面日志里的报错发出来。其余部分（后端、CARLA 接口、同步算法）与 Ubuntu 完全相同，已在 Ubuntu 上完整测试。

---

## 目录

1. [准备工作](#1-准备工作)
2. [推荐的目录结构](#2-推荐的目录结构)
3. [获取代码](#3-获取代码)
4. [安装 CARLA（原版）](#4-安装-carla原版)
5. [安装 Python 和依赖](#5-安装-python-和依赖)
6. [获取界面程序](#6-获取界面程序)
7. [启动](#7-启动)
8. [接入 CarSim（重点）](#8-接入-carsim重点)
9. [改版 CARLA（可选）](#9-改版-carla可选)
10. [命令行与强化学习](#10-命令行与强化学习)
11. [常见问题](#11-常见问题)

---

## 1. 准备工作

| 项目 | 要求 |
|---|---|
| 系统 | Windows 10 / 11 64 位 |
| 显卡 | 支持 DirectX 11 的独立显卡，显存 ≥ 6 GB（推荐 NVIDIA 8 GB 以上），驱动更新到最新 |
| 磁盘 | 只用原版 CARLA：约 30 GB。要编译改版 CARLA：再加约 200 GB |
| 内存 | ≥ 16 GB |
| CarSim | 你已有的 CarSim（含许可证），64 位求解器 |

需要安装的软件：
- **Python 3.10 64 位**：<https://www.python.org/downloads/release/python-31011/>，选 “Windows installer (64-bit)”。
  安装时**勾选 “Add python.exe to PATH”**。（CARLA 0.9.16 也支持 3.11 / 3.12，但 3.10 最稳妥。）
- **Git for Windows**：<https://git-scm.com/download/win>（一路默认安装即可）。
- **7-Zip**（可选，解压大文件更快）：<https://www.7-zip.org/>。

安装完打开“命令提示符”（按 Win 键搜索 `cmd`）检查：
```bat
python --version        rem 应显示 Python 3.10.x
git --version
```

---

## 2. 推荐的目录结构

脚本和程序默认按下面的结构找文件，**照这个放就不用改任何配置**：
```
C:\carla-cosim-studio\                     ← 本仓库（路径里最好不要有中文和空格）
├── CARLA_0.9.16\                          ← 第 4 步解压的原版 CARLA
├── CARLA_CoSim_Studio_Windows\            ← 第 6 步解压的界面程序
│   ├── carla_cosim_studio.exe
│   └── fonts\
├── python_carsim_env\                     ← CarSim 的 Python 接口
├── venv\                                  ← 第 5 步创建的 Python 环境
├── carsim_carla_bridge\                   ← 后端
├── scripts\windows\                       ← 一键启动脚本
└── cosim_gui\  carla_patches\  docs\
```

---

## 3. 获取代码

打开“命令提示符”：
```bat
cd /d C:\
git clone https://github.com/Mrchengyuan/carla-cosim-studio.git
cd carla-cosim-studio
git clone https://github.com/Mrchengyuan/python_carsim_env.git   # 原始仓库：https://github.com/dyZhou2001/python_carsim_env（原作者 dyZhou2001）
```
> 仓库是私有的话，Git 会弹出登录窗口，用 GitHub 账号登录（或用户名 + Personal Access Token）。
> 不想用 Git 的话，也可以在 GitHub 页面点 **Code → Download ZIP**，解压到 `C:\carla-cosim-studio`。

---

## 4. 安装 CARLA（原版）

原版 CARLA 不用编译，下载解压就能用。界面会自动使用“兼容模式”：车身、转向、车轮转动完全同步，只是速度 / IMU 读数为 0，也没有悬架动画（要这些见第 9 步）。

1. 下载 <https://downloads.carlasim.com/Windows/CARLA_0.9.16.zip>（约 8 GB）。
2. 解压到 `C:\carla-cosim-studio\CARLA_0.9.16\`。
   解压后，`CarlaUE4.exe` 在 `CARLA_0.9.16\CarlaUE4.exe` 或 `CARLA_0.9.16\WindowsNoEditor\CarlaUE4.exe`，两种都可以，脚本会自动识别。
3. 试运行：双击 `CarlaUE4.exe`，等 10–30 秒出现城市画面即正常。
   - 第一次运行时 **Windows 防火墙** 可能会弹窗，请点“允许访问”（至少勾选“专用网络”），否则界面连不上 CARLA。
   - 关闭 CARLA：直接关窗口。
4. 可选：额外地图 `AdditionalMaps_0.9.16.zip`，解压到 CARLA 目录后按其说明导入。

---

## 5. 安装 Python 和依赖

在“命令提示符”里执行：
```bat
cd /d C:\carla-cosim-studio
python -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install numpy pillow shapely networkx
```
然后安装 CARLA 的 Python 包，**二选一**：
```bat
rem 方法 A：用 CARLA 安装包自带的 wheel（在 PythonAPI\carla\dist 里，选 cp310 + win_amd64 的那个）
dir /s /b CARLA_0.9.16\*.whl
pip install <上面列出的 carla-0.9.16-cp310-cp310-win_amd64.whl 的完整路径>

rem 方法 B：直接从 PyPI 安装
pip install carla==0.9.16
```
检查：
```bat
python -c "import carla; print(carla.__file__)"
```
> 国内下载慢的话，在 pip 命令后面加 `-i https://pypi.tuna.tsinghua.edu.cn/simple`。

说明：“路线跟随”驾驶模式用到 CARLA 的路径规划模块（`agents`），它在 `CARLA_0.9.16\PythonAPI\carla` 里，不在 pip 包里。按第 2 步的目录结构放置时会自动找到；放在别处的话，在 `scripts\windows\env.bat` 里设置 `CARLA_PYTHONAPI`。

---

## 6. 获取界面程序

**方法 A：直接下载（推荐）**
1. 打开仓库的 **Releases** 页面，下载 `CARLA_CoSim_Studio_Windows.zip`。
2. 解压到 `C:\carla-cosim-studio\`，得到 `C:\carla-cosim-studio\CARLA_CoSim_Studio_Windows\carla_cosim_studio.exe`。
   **`fonts` 文件夹必须和 exe 放在一起**（里面是图标字体）。
3. 第一次运行时，Windows 可能提示“Windows 已保护你的电脑”（因为程序没有数字签名）：点 **更多信息 → 仍要运行**。

**方法 B：自己编译**（需要 Visual Studio 2022，安装时勾选“使用 C++ 的桌面开发”，以及 CMake）
在 “x64 Native Tools Command Prompt for VS 2022” 里：
```bat
cd /d C:\carla-cosim-studio\cosim_gui
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```
生成 `cosim_gui\build\Release\carla_cosim_studio.exe`（`fonts` 文件夹会自动复制到旁边）。一键启动脚本也会自动找到这个位置。

---

## 7. 启动

### 7.1 方式一：桌面一键启动（推荐）

先创建桌面快捷方式（只需一次）：双击 `C:\carla-cosim-studio\scripts\windows\install_shortcuts.bat`。
桌面会出现 3 个快捷方式：

| 快捷方式 | 作用 |
|---|---|
| **CARLA CoSim Studio** | 启动原版 CARLA（离屏，不弹 CARLA 窗口），然后打开界面并自动连接 |
| **CARLA CoSim Studio (mod)** | 启动改版 CARLA（需要第 9 步） |
| **Stop CARLA** | 关闭所有 CARLA，释放显存 |

使用：双击 **CARLA CoSim Studio** → 等 10–30 秒 → 界面打开并自动连上 CARLA → 用完**直接关闭界面**，CARLA 会自动关闭。

路径和设置都在 `scripts\windows\env.bat` 里，用记事本打开即可修改：

| 变量 | 默认值 | 含义 |
|---|---|---|
| `CARLA_ROOT` | `C:\carla-cosim-studio\CARLA_0.9.16` | 原版 CARLA 目录 |
| `CARLA_MOD_ROOT` | `C:\carla\Build\UE4Carla\0.9.16\WindowsNoEditor` | 改版 CARLA 打包目录 |
| `STUDIO_EXE` | `...\CARLA_CoSim_Studio_Windows\carla_cosim_studio.exe` | 界面程序 |
| `COSIM_PYTHON` | `...\venv\Scripts\python.exe` | 后端使用的 Python |
| `CARLA_PYTHONAPI` | `%CARLA_ROOT%\PythonAPI\carla` | CARLA 路径规划模块 |
| `CARLA_PORT` / `CARLA_MOD_PORT` | 2000 / 3000 | 端口 |

### 7.2 方式二：手动启动
1. 双击 `CARLA_0.9.16\CarlaUE4.exe`，等城市画面出现。
2. 双击 `CARLA_CoSim_Studio_Windows\carla_cosim_studio.exe`。
3. 在界面的 **连接** 页：
   - Python 解释器：`C:\carla-cosim-studio\venv\Scripts\python.exe`
   - 桥接目录：`C:\carla-cosim-studio\carsim_carla_bridge`（一般会自动填好）
   - 点 **保存设置**，再点 **连接 CARLA**。
4. 用完：关闭界面，再关闭 CARLA 窗口。

命令行参数（写在快捷方式的“目标”后面）：`--python <路径>`、`--backend-dir <目录>`、`--carla-port 2000`、`--auto-connect`、`--config <json>`、`--light`、`--font <字体文件>`、`--size 2560x1440`（窗口大小）、`--scale 1.5`（界面缩放，高分屏用）。

---

## 8. 接入 CarSim（重点）

CarSim 通过 `python_carsim_env`（调用 CarSim 求解器 DLL）和 CARLA 同步。**先确认 `python_carsim_env` 自己能单独跑通**，这是后面所有步骤的前提。

### 8.1 确认 python_carsim_env 能用
```bat
cd /d C:\carla-cosim-studio\python_carsim_env
..\venv\Scripts\activate
pip install torch gymnasium tensorboard          rem python_carsim_env 自己的依赖（只做联合仿真可不装 torch）
rem 把你的 simfile.sim 复制到这个目录，然后：
python carsim_env.py                              rem carsim_env.py 默认读取当前目录下的 simfile.sim
```
- 必须用 **64 位 Python**，对应 CarSim 的 64 位求解器（`carsim_64.dll`）。
- CarSim **许可证服务**要在运行。如果报许可证错误，先在 CarSim 界面里正常运行一次仿真确认许可证可用。
- `simfile.sim` 由 CarSim 生成，里面的 `PROGDIR` / `DLLFILE` 指向求解器。用你之前在 python_carsim_env 里已经跑通的那个 `.sim` 就行（仓库里不带 `.sim`，需要从你的 CarSim 数据库生成 / 复制）。

### 8.2 在 CarSim 里配置导出变量
在 CarSim 的 Import/Export 界面（I/O Channels: Export）按
[CarSim 导出变量清单](../carsim_carla_bridge/docs/CarSim导出变量清单.md) 添加变量：
- **必需**：`Xo Yo Zo Yaw Pitch Roll Steer_L1 Steer_R1`
- **推荐**：`Vx Vy AVx AVy AVz`（真实速度 / IMU）、`AVy_L1 AVy_R1 AVy_L2 AVy_R2`（车轮转速，能看到打滑）、`Jnc_L1 Jnc_R1 Jnc_L2 Jnc_R2`（悬架行程）、`Steer_SW Throttle GearStat`、`Steer_L2 Steer_R2`（后轮转向）

**记下变量的顺序**，界面里要按同样的顺序填。
Import（输入）保持你原来的三个，REPLACE 模式：油门、制动、方向盘转角（度），和 python_carsim_env 一致。你的控制算法算出的值就是写进这三个变量。

### 8.3 在界面里配置
1. **车辆与视角**：点“测量全部车型尺寸”，挑一款轴距、轮胎半径和你的 CarSim 车型接近的车；选出生点（CarSim 原点放在这里）。
2. **CarSim 动力学**：
   - 取消“模拟 CarSim”。
   - `.sim 文件`：例如 `C:\carla-cosim-studio\python_carsim_env\simfile.sim`。
   - `python_carsim_env 目录`：`C:\carla-cosim-studio\python_carsim_env`。
   - 导出变量：按 8.2 的顺序填（可以从 CarSim 复制后“用粘贴内容替换”），看到绿色“必需变量齐全”。
   - 单位：CarSim 默认用户单位（deg、km/h、deg/s、rpm、mm），一般不用改。
3. **驾驶模式**：选 **CarSim 联合仿真**，下面的“控制算法”里：
   - **算法文件 .py**：填你的控制算法文件，例如 `controllers\my_controller.py`（相对 `carsim_carla_bridge` 目录）或绝对路径 `D:\my_algo\my_controller.py`。
   - **入口**：类名，默认 `Controller`。
   - 算法文件的写法（复制 `controllers\example_controller.py` 改最方便）：

     ```python
     # 例：C:\carla-cosim-studio\carsim_carla_bridge\controllers\my_controller.py
     class Controller:
         def reset(self):                       # 可选，每次运行开始调用一次
             self.integral = 0.0

         def control(self, exports, t, dt):     # 每帧调用一次
             vx = exports["Vx"]                 # 按变量名取 CarSim 导出变量（CarSim 单位）
             ...                                # 你的算法
             return [throttle, brake, steer_sw] # 按 .sim 里导入变量的顺序
     ```

     `exports` 按变量名取 CarSim 导出变量；返回值按 .sim 里导入变量的顺序。每次点“运行”都会重新加载文件，改完代码直接再运行。
     想用原来的 SimplePathFollower，就填 `controllers\simple_path_follower.py`（需要导出 `LatErr`）。
   - 仿真步长 = 控制周期，用 CarSim `t_step` 的整数倍，例如 `t_step = 0.001` 时用 `0.02`。
   - “测试用驾驶方式”折叠栏里的演示 / 路线跟随 / 键盘驾驶，只在还没有算法、想先检查链路时用。
4. 把 **日志 CSV** 填上（例如 `cosim_log.csv`），点顶部 **运行**。
5. 检查同步：打开“实时画面”的“前轮特写”，看转向和车轮转动；运行结束后打开日志 CSV，`carsim_x/y/yaw` 和 `carla_x/y/yaw` 应该一一对应。

### 8.4 参考点和坐标
- CarSim 的 `Xo, Yo, Zo` 默认是前轴中心、地面高度；程序会按 CARLA 车型的前轴位置自动换算。静止时 `Zo` 应接近 0，否则在“CarSim 动力学”页修改参考点。
- 坐标换算（ISO → CARLA）程序自动处理：`y → -y`，`yaw → -yaw`，`pitch → -pitch`，车轮转角取反。
- 如果 CarSim 路面和 CARLA 地图高度不一致，把“高度模式”改成“贴合 CARLA 路面”。

---

## 9. 改版 CARLA（可选）

改版 CARLA 增加外部动力学接口：一帧一次下发完整状态，`get_velocity()` / IMU 读数是真实值，有悬架动画。
**编译步骤见 [Windows 编译指南](../carsim_carla_bridge/docs/Windows编译指南.md)**（需要 VS 2022、关联 Epic 的 GitHub 账号、约 200 GB 磁盘和几个小时）。概要：

1. 编译 CARLA 定制版 UE4（`Setup.bat`、`GenerateProjectFiles.bat`，VS 2022 编译 `UE4.sln`），设置环境变量 `UE4_ROOT`。
2. 克隆 CARLA 0.9.16：`git clone --depth 1 -b 0.9.16 https://github.com/carla-simulator/carla.git C:\carla`
3. 打补丁：`cd /d C:\carla` → `git apply C:\carla-cosim-studio\carla_patches\carla_0.9.16_external_dynamics.patch`
   （`linux_libpng_url_fix.patch` 只在 Linux 需要）。
4. 在 “x64 Native Tools Command Prompt for VS 2022” 里：`Update.bat` → `make PythonAPI` → `make package`。
5. 打包结果在 `C:\carla\Build\UE4Carla\0.9.16\WindowsNoEditor\`，这正是 `env.bat` 里 `CARLA_MOD_ROOT` 的默认值。
6. 安装改版 Python 包：`pip install C:\carla\PythonAPI\carla\dist\carla-0.9.16-cp310-cp310-win_amd64.whl --force-reinstall`，
   检查 `python -c "import carla; print(hasattr(carla.Vehicle, 'apply_external_state'))"` 输出 `True`。
7. 双击 **CARLA CoSim Studio (mod)**；界面“连接”页会显示绿色的“改版 CARLA：可用”。
8. 验证：`python carsim_carla_bridge\tests\test_modified_carla.py --port 3000`，应该 11 项全部 PASS。

---

## 10. 命令行与强化学习

```bat
cd /d C:\carla-cosim-studio\carsim_carla_bridge
..\venv\Scripts\activate
python run_cosim.py --mock --duration 20                                       rem 模拟 CarSim
python run_cosim.py --config ..\cosim_config.json                              rem 界面保存的配置
python run_cosim.py --sim C:\carla-cosim-studio\python_carsim_env\simfile.sim --carsim-repo ..\python_carsim_env --controller controllers\my_controller.py
```
在训练代码里使用（每个 `env.control_step()` 后加两行）：
```python
from bridge import CarlaVehicleSync
sync = CarlaVehicleSync(world, vehicle, anchor_transform)      # 初始化一次
obs, r, done, info = env.control_step(action, inner_steps)
sync.sync(obs, env.t_current, frame_dt)
world.tick()
```

---

## 11. 常见问题

| 现象 | 原因和解决 |
|---|---|
| 双击 exe 提示“Windows 已保护你的电脑” | 程序没有数字签名：点“更多信息 → 仍要运行” |
| 界面上图标显示成方框 | `fonts` 文件夹没有和 exe 放在一起 |
| 中文显示异常 | 程序使用系统自带的微软雅黑（`C:\Windows\Fonts\msyh.ttc`）；精简版系统可用 `--font` 指定其他中文字体 |
| 状态栏“后端 未运行” | Python 路径不对或缺包。看 `carsim_carla_bridge\backend.log`；在“连接”页填 `venv\Scripts\python.exe` 的完整路径 |
| 连接 CARLA 超时 | CARLA 没启动好，或被防火墙拦截：控制面板 → Windows Defender 防火墙 → 允许应用 → 勾选 CarlaUE4 |
| 提示 57100 端口被占用 | 上次的后端还在：任务管理器结束 `python.exe`，或 `netstat -ano \| findstr 57100` 找到进程号后 `taskkill /PID <号> /F` |
| `No module named agents` | 找不到 CARLA 路径规划模块：在 `env.bat` 设置 `CARLA_PYTHONAPI` |
| CarSim 报找不到 DLL / 许可证错误 | 用 64 位 Python；确认 `.sim` 里的求解器路径正确、许可证服务在运行；先单独跑通 `python_carsim_env` |
| 车辆在 CARLA 里浮空或陷进地面 | 参考点或高度不一致：“CarSim 动力学”页改参考点，或把高度模式改成“贴合 CARLA 路面” |
| 车辆方向反了 / 转向反了 | 检查 CarSim 导出的角度单位（deg / rad）和导出变量顺序 |
| 车停着不动 | ① 工具栏显示“已完成”、画面上方有黄色提示条：运行时长到了，把“驾驶模式 → 运行时长”设为 0；② CARLA 自动驾驶在等红灯（视口上方有提示），变绿后会自己走；③ CarSim 的制动输入比例太大。“路线跟随”不看红绿灯 |
| CARLA 关了显存还被占 | 双击 **Stop CARLA** |
| 高分辨率屏界面太小 / 太大 | 右键 exe → 属性 → 兼容性 → 更改高 DPI 设置 |
