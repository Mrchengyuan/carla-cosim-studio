"""多拓扑非线性 MPC（AL-iLQR 求解）：规划和控制在同一个优化里——跟车道、过弯限速、跟车、换道避障、红灯停车。

思路
    预测模型   Frenet 坐标（沿车道中心线的弧长 s、横向偏差 ey）下的动态自行车 + Pacejka 非线性侧向轮胎
               + 转向、纵向执行器的一阶滞后（nmpc_model.py）。时域 N × DT_MPC = 4 s。
    优化       增广拉格朗日 iLQR（nmpc_solver.py）：Gauss-Newton 反向 Riccati 递推，不等式约束用增广拉格朗日
               （乘子跨周期继承），雅可比用有限差分批量算。每个控制周期热启动迭代 ITERS 次（实时迭代）。
    约束       输入：ax ∈ [AX_MIN, AX_MAX]、前轮转角、转角速率；状态：轮胎摩擦圆（纵向 + 侧向加速度
               ≤ FRICTION_USE × μg）、车道边界（只能开在同向车道上）、车速 ≥ 0；
               每个障碍物：按它现在的速度沿车道外推，自车用 3 个圆覆盖，障碍物膨胀成椭圆，
               每个时刻都不能相交；红 / 黄灯：车头不越过停止线。
    多拓扑     绕障碍物从左还是从右过、换不换道，是非凸的：一次优化只会落到其中一个局部最优。
               所以“本车道 / 换到左边车道 / 换到右边车道”几个方案（拓扑类）并行优化（批量一起算），
               选 满足约束 且 代价最低 的那个；换方案要多付 SWITCH_COST（防止来回犹豫），
               离开出发车道每条车道多付 HOME_COST（超过去以后自己换回来）。
    跟车、避让  不用另写规则：前车慢，本车道方案要么减速（车速代价大），要么违反约束；
               换道方案不用减速，代价低就自然选它。左右都走不了，就在本车道上减速、停下。
    在线自适应  ADAPT = True 时用递推最小二乘（带遗忘因子）在线估计前、后轴轮胎侧向力的比例因子
               （实测侧向加速度、横摆角加速度反推出的轴侧向力 ÷ 模型的轴侧向力），模型跟着车变。
    安全层     每帧按 RSS（Mobileye 责任敏感安全模型）检查本车道前方目标的纵向安全距离；
               不够时不管优化器怎么说都至少按 RSS_BRAKE 制动（优化器出问题时的最后一道保险）。

输入（“场景信息”页勾选给算法）
    车道   center_rel、width、offset（默认已勾），left_lane、right_lane（要勾上才会换道），
           light_state、light_dist（要勾上才看红绿灯）
    障碍物 rel_x、rel_y、rel_yaw、length、width、type，最好再勾 Vx_global、Vy_global（没有时用 rel_vx、rel_vy）
    自车   X、Y、Yaw（路口里记住路径用）、length、width
    导出   Vx；有 Vy、AVz 时用实测侧向车速、横摆角速度（没有时按运动学估计）；
           Steer_L1、Steer_R1（实际前轮转角，没有时按模型估计）
    单位跟随“CarSim 动力学”页（scene["units"]）。

输出
    OUTPUT = "ax_delta"：[纵向加速度 ax (m/s²), 前轮转角 (rad，左为正)]——Chrono 宝马替身的导入。
    OUTPUT = "carsim"：[油门 0~1, 制动（主缸压力 MPa）, 方向盘转角 deg]——真实 CarSim（换算同 KMPPI 的 carsim 输出）。

画线（CARLA 画面和界面“轨迹”页）：绿色粗线 = 选中的方案，橙色 = 其他满足约束的方案，红色 = 违反约束的方案，
    蓝色细线 = 参考车道中心线，白色 = 障碍物在时域末端的预测位置。
曲线（界面“曲线”页、log_debug.csv）：方案、代价、约束违反、求解耗时、ax、前轮转角、轮胎比例因子、RSS 介入。
"""
import json
import math
import os
import time

import numpy as np

from nmpc_model import NX, NU, S, EY, EPSI, VX, VY, R, DELTA, AX, DC, Vehicle
from nmpc_reference import LaneMemory, Path, obstacles_frenet
from nmpc_solver import ALiLQR

FRAME_DT = 0.05          # s，仿真步长（平台自动使用）：每帧优化一次
OUTPUT = "ax_delta"      # "ax_delta"：Chrono 宝马；"carsim"：真实 CarSim（[油门, 制动, 方向盘转角]）
TARGET_KMH = 72.0        # km/h，期望车速
A_LAT_MAX = 3.0          # m/s²，弯道限速用的侧向加速度
A_DEC_COMFORT = 2.0      # m/s²，限速变化（进弯、停车线）提前减速用的减速度
MU = 0.9                 # 路面附着系数
FRICTION_USE = 0.8       # 最多用到附着极限的这么多
AX_MAX, AX_MIN = 2.5, -7.0   # m/s²，纵向加速度指令范围
DELTA_MAX = 0.35         # rad，前轮转角
DELTA_RATE_MAX = 0.6     # rad/s，前轮转角速率
LANE_CHANGE = True       # 允许换道（False：只在本车道上跟车、停车）
HOME_COST = 80.0         # 离开出发车道，每条车道的代价（超车后换回来）
SWITCH_COST = 40.0       # 换方案的代价（防止来回犹豫）
FOLLOW_TIME = 1.2        # s，跟车时距：和前车（横向有重叠时）至少保持 2 m + 自车车速 × 它
REAR_TIME = 1.0          # s，换道时给后方来车留的时距：它后面至少 2 m + 它的车速 × 它
STOP_GAP = 3.0           # m，停在静止障碍物（锥桶、停着的车、行人）前多远
SIDE_MARGIN = 0.35       # m，和车辆、行人横向至少留这么多
SIDE_MARGIN_STATIC = 0.2 # m，和锥桶、护栏等静止的小东西横向至少留这么多
LANE_MARGIN = 0.15       # m，车身离同向车道外边界至少这么多
STOP_MARGIN = 2.0        # m，车头停在停止线前这么远
N_STEPS = 40             # 预测步数
DT_MPC = 0.1             # s，预测步长
ITERS = 3                # 每个控制周期的 iLQR 迭代次数
ITERS_COLD = 6           # 有方案从头开始（没有热启动）的那个周期多迭代几次
ITERS_FIRST = 15         # 第一个周期（所有方案都从头开始）迭代次数
MAX_OBS = 16             # 最多考虑几个障碍物（按距离）
N_CIRCLES = 4            # 自车用几个圆覆盖（越多越贴合车身，计算量越大）
COMMIT_COST = 400.0      # 换道开始以后中途放弃的代价（目标车道还走得通时不轻易取消）
ADAPT = True             # 在线估计轮胎比例因子
RSS_REACTION = 0.3       # s，RSS 反应时间
RSS_ACCEL = 2.0          # m/s²，RSS 反应时间内自车最大加速度
RSS_BRAKE = 6.0          # m/s²，RSS 自车最小制动减速度（不满足安全距离时至少这么制动）
RSS_LEAD_BRAKE = 8.0     # m/s²，RSS 前车最大制动减速度
RSS_PLAN_T = 1.0         # s，计划的轨迹在这段时间内横向离开前车所在范围（正在换道绕开）时，RSS 不介入
VIOL_TOL = 0.05          # 约束违反量在这以内算满足（约束都已归一化）
EARLY_T = 2.5            # s，这段时间内的约束违反 = 方案走不通（更远的只加罚 LATE_PENALTY × 违反量）
LATE_PENALTY = 3000.0
EMERGENCY_FRAMES = 2     # 连续这么多帧所有方案都走不通才全力制动（单帧的求解起伏不算）
ABORT_FRAMES = 6         # 换道中目标车道连续这么多帧走不通才放弃
DWELL = 2.0              # s，换完一次道后这么久内不再开始新的换道（本车道走不通时除外）
RESET_FRAMES = 6         # 方案连续这么多帧走不通：丢掉它的热启动重新开始
RELAX_T = 2.5            # s，车在方案的横向范围外时，范围从车现在的位置起这么久收回来
SCORE_FILTER = 0.3       # 方案代价的一阶滤波系数（每帧新值占的比例）：单帧求解的起伏不会让决策来回跳
RIGHT_PASS_COST = 400.0  # 往右换道（不是回出发车道方向）多付的代价：超车走左侧（右边是唯一出路时才往右）
VEHICLE_JSON = ""        # 车辆参数文件（“运行对比”页的车辆参数辨识；空 = Chrono 宝马 E90）
TAU_DELTA = 0.10         # s，转向执行器时间常数
TAU_AX = 0.20            # s，纵向执行器时间常数
# OUTPUT = "carsim" 时的换算（同 KMPPI）：
STEER_RATIO_INIT = 19.0
BRAKE_MAX = 8.0
ACCEL_FF = 0.1
KP, KI = 0.08, 0.02
V_TARGET_SLACK = 2.0
STANDSTILL_MS = 1.0      # m/s，OUTPUT = "ax_delta" 时：低于这个车速，制动按车速收小（停住不倒车）
STANDSTILL_STEER_MS = 0.3  # m/s，低于这个车速方向盘不动
HOLD_GAIN = 3.0          # 1/s，停车保持：ax = −HOLD_GAIN × 车速
PRINT_EVERY = 5.0
DRAW = True

# 代价权重（残差的系数；代价 = ½ Σ 残差²）
W_EY, W_EY_T = 3.0, 4.0
W_EY_LC = 1.2            # 换道方案的横向偏差权重（小一些：换道不急）
W_EPSI, W_EPSI_T = 2.0, 6.0
W_V, W_V_T = 0.6, 1.0
W_AY = 0.6
W_AX, W_JERK = 0.3, 0.3
W_DRATE = 6.0
W_DELTA = 1.0

LANE_KINDS = ("same",)
EGO_LEN, EGO_WID = 4.6, 1.85   # m，场景没给自车尺寸时用


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class _Problem:
    """这一周期的优化问题（solver.py 要的接口）。"""

    def __init__(self, ctl, path, hyps, vref_s, vref_v, obs, stop_s):
        self.c = ctl
        self.veh = ctl.veh
        self.path = path
        self.H = len(hyps)
        self.N = N_STEPS
        # 每个方案：目标横向位置、可走的横向范围（拓扑类：本车道方案只能在本车道里）、参考车速倍数
        self.targets = np.array([hp["target"] for hp in hyps], dtype=float)
        # 横向范围按时刻放宽：车现在就在范围外（刚换进新车道、被挤到边上）时，从现在的位置起 RELAX_T 秒内收回到范围里
        L, W = ctl.ego_len, ctl.ego_wid
        ey0 = ctl.x0[EY]
        fade = np.clip(1.0 - np.arange(N_STEPS + 1) * DT_MPC / RELAX_T, 0.0, 1.0)
        lb = np.array([hp["left"] for hp in hyps], dtype=float)[:, None]
        rb = np.array([hp["right"] for hp in hyps], dtype=float)[:, None]
        over_l = np.maximum(0.0, ey0 + W / 2.0 - (lb - LANE_MARGIN) + 0.1)
        over_r = np.maximum(0.0, (rb + LANE_MARGIN) - (ey0 - W / 2.0) + 0.1)
        self.left_b = lb + over_l * fade[None, :]          # (H, N+1)
        self.right_b = rb - over_r * fade[None, :]
        self.vscale = np.array([hp["vscale"] for hp in hyps], dtype=float)
        self.nominal = False                    # True：按正常参考车速算代价（选方案时比较用）
        self.vref_s, self.vref_v = vref_s, vref_v
        self.stop_s = stop_s
        L, W = ctl.ego_len, ctl.ego_wid
        self.half_w = W / 2.0
        self.front = L / 2.0
        # 自车用 N_CIRCLES 个圆覆盖（圆心沿车身纵轴均匀分布）
        span = max(0.0, L - W)
        self.circ = np.linspace(-span / 2, span / 2, N_CIRCLES)
        step = span / (N_CIRCLES - 1)
        self.rad = math.hypot(W / 2.0, step / 2.0)
        # 障碍物在每个预测时刻的位置（常速外推）：(N+1, M)
        M = MAX_OBS
        t = np.arange(N_STEPS + 1)[:, None] * DT_MPC
        self.os = np.full((N_STEPS + 1, M), 1e4)
        self.od = np.zeros((N_STEPS + 1, M))
        self.oA = np.ones(M)
        self.oB = np.ones(M)
        self.ahead = np.ones(M)       # 1：在前方（跟它），0：在后方（别切到它前面）
        self.ohs = np.zeros(M)        # 沿车道方向的半长
        self.olat = np.ones(M)        # 横向“同一车道”判定的半宽
        self.gfix = np.zeros(M)       # 纵向距离要求：固定部分 m
        self.gtime = np.zeros(M)      # 纵向距离要求：× 自车车速（前方）的时距 s
        self.rear = L / 2.0
        tt = t[:, 0]
        for j, o in enumerate(obs[:M]):
            self.os[:, j] = o.s + o.vs * tt
            if o.kind == "walker":            # 行人：常速
                self.od[:, j] = o.d + o.vd * tt
            else:                             # 车辆：横向速度在 1.5 s 左右消失（换完道就沿车道走了）
                self.od[:, j] = o.d + o.vd * 1.5 * (1.0 - np.exp(-tt / 1.5))
            # 碰撞：贴身的椭圆
            self.oA[j] = o.half_s + self.rad + 0.5
            self.oB[j] = o.half_d + self.rad + (SIDE_MARGIN_STATIC if o.kind == "static" else SIDE_MARGIN)
            # 纵向距离（只在横向有重叠时起作用）
            self.ohs[j] = o.half_s
            self.olat[j] = o.half_d + self.half_w
            moving = o.kind == "vehicle" and abs(o.vs) > 1.0
            if o.s >= 0:                      # 前方：跟车 2 m + 时距 × 自车车速；静止的东西停在 3 m 外
                self.ahead[j] = 1.0
                self.gfix[j], self.gtime[j] = (2.0, FOLLOW_TIME) if moving else (STOP_GAP, 0.0)
            else:                             # 后方：给它留 2 m + 它的车速 × REAR_TIME（换道时别切到快车前面）
                self.ahead[j] = 0.0
                self.gfix[j] = 2.0 + REAR_TIME * max(o.vs, 0.0)
        # 横向偏差权重：本车道方案从头就是 W_EY（贴着中心线走）；换道方案近处轻、远处重（换道平顺）
        ramp = np.clip(np.arange(N_STEPS) * DT_MPC / 2.5, 0.15, 1.0)
        self.w_ey = np.where(np.abs(self.targets)[:, None] > 0.1, ramp[None, :] * W_EY_LC, W_EY)   # (H, N)
        self.kap = path.kappa

    def dyn(self, x, u):
        return self.veh.step(x, u, DT_MPC, self.kap)

    def vref(self, s):
        return np.interp(s, self.vref_s, self.vref_v)

    def res(self, x, u, k, h):
        ey_t = self.targets[h]
        # 舒适：侧向加速度超出“沿参考线过弯本来就要的”那部分（换道、躲避时的侧向加速度）
        ay = x[..., VX] * x[..., R] - x[..., VX] ** 2 * self.kap(x[..., S])
        return np.stack([
            self.w_ey[h, k] * (x[..., EY] - ey_t),
            W_EPSI * x[..., EPSI],
            W_V * (x[..., VX] - self.vref(x[..., S]) * (1.0 if self.nominal else self.vscale[h])),
            W_AY * ay,
            W_AX * u[..., 0],
            W_JERK * (u[..., 0] - x[..., AX]) / TAU_AX,
            W_DRATE * u[..., 1],
            W_DELTA * x[..., DC],
        ], axis=-1)

    def res_T(self, x, h):
        return np.stack([
            W_EY_T * (x[..., EY] - self.targets[h]),
            W_EPSI_T * x[..., EPSI],
            W_V_T * (x[..., VX] - self.vref(x[..., S]) * (1.0 if self.nominal else self.vscale[h])),
            W_EPSI_T * 0.5 * x[..., R],
        ], axis=-1)

    def _state_con(self, x, k, h):
        g = MU * self.veh.g * FRICTION_USE
        ay = x[..., VX] * x[..., R]
        cs = [((x[..., AX] / g) ** 2 + (ay / g) ** 2) - 1.0,
              (x[..., EY] + self.half_w - (self.left_b[h, k] - LANE_MARGIN)),
              ((self.right_b[h, k] + LANE_MARGIN) - (x[..., EY] - self.half_w)),
              -x[..., VX] / 5.0]
        ce, se = np.cos(x[..., EPSI]), np.sin(x[..., EPSI])
        os_k, od_k = self.os[k], self.od[k]                                 # (..., M)
        for l in self.circ:
            sc = (x[..., S] + l * ce)[..., None]
            dc = (x[..., EY] + l * se)[..., None]
            cs.append(1.0 - ((sc - os_k) / self.oA) ** 2 - ((dc - od_k) / self.oB) ** 2)
        # 跟车 / 后方来车的纵向距离，按横向重叠程度（平滑的 0~1）加权
        dd = np.sqrt((x[..., EY][..., None] - od_k) ** 2 + 0.01)
        w = 1.0 / (1.0 + np.exp(np.clip((dd - self.olat) / 0.1, -50.0, 50.0)))
        s_e = x[..., S][..., None]
        need_f = self.gfix + self.gtime * np.maximum(x[..., VX], 0.0)[..., None]
        c_ahead = (s_e + self.front + need_f) - (os_k - self.ohs)
        c_rear = (os_k + self.ohs + self.gfix) - (s_e - self.rear)
        cs.append(np.where(self.ahead > 0.5, c_ahead, c_rear) / 10.0 * w)
        if self.stop_s is not None:
            cs.append((x[..., S] + self.front - (self.stop_s - STOP_MARGIN))[..., None] / 2.0)
        out = []
        for c in cs:
            out.append(c if c.ndim == x.ndim else c[..., None])
        return np.concatenate(out, axis=-1)

    def con(self, x, u, k, h):
        ui = np.stack([(u[..., 0] - AX_MAX) / 2.0, (AX_MIN - u[..., 0]) / 2.0,
                       (x[..., DC] - DELTA_MAX) / 0.1, (-DELTA_MAX - x[..., DC]) / 0.1,
                       (u[..., 1] - DELTA_RATE_MAX) / 0.2, (-DELTA_RATE_MAX - u[..., 1]) / 0.2], axis=-1)
        return np.concatenate([ui, self._state_con(x, k, h)], axis=-1)

    def con_T(self, x, h):
        k = np.full(x.shape[:-1], N_STEPS, dtype=int)
        return self._state_con(x, k, h)

    @property
    def nc(self):
        return 6 + 4 + (N_CIRCLES + 1) * MAX_OBS + (1 if self.stop_s is not None else 0)


class Controller:
    def reset(self):
        here = os.path.dirname(os.path.abspath(__file__))
        self.veh = Vehicle(tau_delta=TAU_DELTA, tau_ax=TAU_AX)
        js = VEHICLE_JSON or os.path.join(here, "bmw_e90_identified.json")
        if not os.path.isabs(js):
            js = os.path.join(here, js)
        if os.path.isfile(js):
            with open(js, encoding="utf-8") as f:
                ident = json.load(f)
            for key, attr in (("m", "m"), ("I", "I"), ("a", "a"), ("b", "b"), ("pac_By", "By"), ("pac_Cy", "Cy"),
                              ("pac_Ey", "Ey"), ("front_lateral_scale", "front_scale"),
                              ("rear_lateral_scale", "rear_scale")):
                if key in ident:
                    setattr(self.veh, attr, float(ident[key]))
        self.scale0 = (self.veh.front_scale, self.veh.rear_scale)
        v = self.veh
        print("多拓扑 NMPC：车辆 m %.0f kg、I %.0f kg·m²、a %.3f m、b %.3f m，轮胎比例因子 前 %.3f 后 %.3f；"
              "时域 %d × %.2f s，每帧迭代 %d 次" % (v.m, v.I, v.a, v.b, v.front_scale, v.rear_scale,
                                                   N_STEPS, DT_MPC, ITERS))
        self.solver = ALiLQR(None, iters=ITERS)
        self.mem = LaneMemory()
        self.lane_idx = 0               # 当前车道（出发车道 = 0，往左 +1）
        self.target_idx = 0             # 选中方案的目标车道
        self.choice = None              # 选中的方案 ("lane" | "yield", 车道)
        self.abort_n = 0
        self.dwell_until = -1.0
        self.score_ema = {}
        self.bad_n = {}
        self.emerg_n = 0
        self.stopping_for = False
        self.x0 = np.zeros(NX)
        self.warm = {}                  # 目标车道 → 热启动 {"U", "lam", "mu", "lamT", "muT", "reg"}
        self.dc = 0.0                   # 前轮转角指令（rad）
        self.ax_est = 0.0               # 实际纵向加速度估计（指令过一阶滞后）
        self.ax_cmd = 0.0
        self.vx_now = 0.0
        self.ego_len, self.ego_wid = EGO_LEN, EGO_WID
        self.draw = None
        self.debug = None
        self.next_print = 0.0
        self.last_t = None
        self.prev_meas = None           # 自适应用：上一帧 (vy, r, t)
        self.P = np.array([0.5, 0.5])   # RLS 协方差
        self.told = set()
        # 统计
        self.n_calls = 0
        self.solve_s = 0.0
        self.solve_max = 0.0
        self.offsets = []
        self.lane_changes = 0
        self.rss_frames = 0
        self.min_gap = None
        self.ratio = STEER_RATIO_INIT
        self.v_target = None
        self.integral = 0.0

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _units(scene):
        u = scene.get("units") or {}
        deg = 1.0 if u.get("angle", "deg") == "deg" else 180.0 / math.pi
        ms = 1.0 / 3.6 if u.get("speed", "km/h") == "km/h" else 1.0
        rate = math.pi / 180.0 if u.get("rate", "deg/s") == "deg/s" else 1.0
        return deg, ms, rate

    def _say(self, key, msg):
        if key not in self.told:
            self.told.add(key)
            print(msg)

    def _measure(self, exports, scene, deg, ms, rate):
        """当前状态（质心）：vx, vy, r, 实际前轮转角。"""
        ego = scene.get("ego") or {}
        if "Vx" in exports:
            vx = float(exports["Vx"]) * ms
        else:
            vx = float(ego.get("Speed", 0.0)) * ms
        L = self.veh.L
        if "Steer_L1" in exports and "Steer_R1" in exports:
            delta = math.radians(0.5 * (exports["Steer_L1"] + exports["Steer_R1"]) * deg)
        else:
            delta = self.dc
        if "AVz" in exports:
            r = float(exports["AVz"]) * rate
        else:
            r = vx * math.tan(delta) / L
        if "Vy" in exports:
            vy = float(exports["Vy"]) * ms - self.veh.a * r   # 参考点（前轴）→ 质心
        else:
            vy = r * self.veh.b
        return vx, vy, r, delta

    def _pose(self, exports, scene, deg):
        e = scene.get("ego") or {}
        for src, keys in ((e, ("X", "Y", "Yaw")), (exports, ("Xo", "Yo", "Yaw"))):
            if all(k in src and src[k] is not None for k in keys):
                return float(src[keys[0]]), float(src[keys[1]]), math.radians(float(src[keys[2]]) * deg)
        return None

    def _adapt(self, vx, vy, r, delta, t):
        """RLS 估计前、后轴轮胎比例因子：实测轴侧向力 = 比例因子 × 模型（比例因子 1）的轴侧向力。"""
        if self.prev_meas is None or vx < 8.0:
            self.prev_meas = (vy, r, t)
            return
        vy0, r0, t0 = self.prev_meas
        dt = t - t0
        self.prev_meas = (vy, r, t)
        if dt <= 1e-4:
            return
        v = self.veh
        vy_dot, r_dot = (vy - vy0) / dt, (r - r0) / dt
        ay = vy_dot + r * vx
        Fyf_m = (v.m * v.b * ay + v.I * r_dot) / (v.L * max(math.cos(delta), 0.5))
        Fyr_m = (v.m * v.a * ay - v.I * r_dot) / v.L
        af, ar = v.slips(np.array(vx), np.array(vy), np.array(r), np.array(delta))
        Df, Dr = v.axle_peak()
        F0 = np.array([Df * v.pacejka(af), Dr * v.pacejka(ar)])
        meas = np.array([Fyf_m, Fyr_m])
        lam = 0.998
        for i in range(2):
            if abs(F0[i]) < 0.03 * (Df if i == 0 else Dr):   # 侧向力太小：没信息
                continue
            th = v.front_scale if i == 0 else v.rear_scale
            K = self.P[i] * F0[i] / (lam + F0[i] * self.P[i] * F0[i])
            th = th + K * (meas[i] - F0[i] * th)
            self.P[i] = min((1 - K * F0[i]) * self.P[i] / lam, 1.0)
            # 限幅、限速：单帧最多变 1%，范围 0.5 ~ 1.5 倍初值
            old = v.front_scale if i == 0 else v.rear_scale
            th = min(old * 1.01, max(old * 0.99, th))
            th = min(1.5 * self.scale0[i], max(0.5 * self.scale0[i], th))
            if i == 0:
                v.front_scale = float(th)
            else:
                v.rear_scale = float(th)

    def _speed_profile(self, path, lane, stop_s):
        s = path.s[path.s >= -1.0]
        kap = np.abs(path.kappa(s))
        vt = TARGET_KMH / 3.6
        v = np.minimum(vt, np.sqrt(A_LAT_MAX / np.maximum(kap, 1e-6)))
        if stop_s is not None:
            v = np.where(s >= stop_s - STOP_MARGIN - self.ego_len / 2.0, 0.0, v)
        for i in range(len(s) - 2, -1, -1):           # 提前减速
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * A_DEC_COMFORT * (s[i + 1] - s[i])))
        return s, v

    def _stop_line(self, lane, vx):
        """红灯 / 黄灯（能舒适停下时）的停止线 s（质心坐标），没有时 None。"""
        st, dist = lane.get("light_state"), lane.get("light_dist")
        if st not in ("red", "yellow") or dist is None:
            self._yellow_go = False
            return None
        s_stop = float(dist) + self.veh.a          # light_dist 从参考点（前轴）量起
        if st == "yellow":
            room = s_stop - STOP_MARGIN - self.ego_len / 2.0
            if getattr(self, "_yellow_go", False) or room <= 0 or vx * vx / (2 * room) > 4.0:
                self._yellow_go = True               # 停不下：黄灯直接通过
                return None
        return s_stop

    def _initial_guess(self, x0, target, prob, brake=False):
        """新方案的初值：一个简单的反馈律（横向 PD 向目标车道、纵向追参考车速）闭环推演出来的输入。"""
        U = np.zeros((N_STEPS, NU))
        x = x0.copy()
        for k in range(N_STEPS):
            # 想要的侧向加速度：二阶系统收敛到目标横向位置（ωn ≈ 0.9 rad/s，阻尼 ≈ 0.9），限 ±A_LAT_MAX；
            # 前轮转角 = 轴距 × (侧向加速度 / v² + 参考线曲率)
            v = max(x[VX], 3.0)
            ay = np.clip(-0.8 * (x[EY] - target) - 1.6 * v * math.sin(x[EPSI]), -A_LAT_MAX, A_LAT_MAX)
            dc_des = np.clip(self.veh.L * (ay / (v * v) + prob.path.kappa(x[S])), -DELTA_MAX, DELTA_MAX)
            U[k, 1] = np.clip((dc_des - x[DC]) / 0.3, -DELTA_RATE_MAX, DELTA_RATE_MAX)
            U[k, 0] = -4.0 if brake else np.clip(0.8 * (prob.vref(x[S]) - x[VX]), AX_MIN, AX_MAX)
            x = prob.dyn(x[None], U[k][None])[0]
        return U

    def _shift(self, arr, el):
        """热启动平移：时域往后挪 el 秒（线性插值，末尾保持）。"""
        if el <= 0:
            return arr.copy()
        n = arr.shape[0]
        tq = np.arange(n) * DT_MPC + el
        tg = np.arange(n) * DT_MPC
        out = np.empty_like(arr)
        flat = arr.reshape(n, -1)
        of = out.reshape(n, -1)
        for j in range(flat.shape[1]):
            of[:, j] = np.interp(tq, tg, flat[:, j])
        return out

    # ------------------------------------------------------------------ control
    def control(self, exports, t, dt, scene):
        t_wall = time.perf_counter()
        deg, ms, rate = self._units(scene)
        lane = scene.get("lane") or {}
        ego = scene.get("ego") or {}
        if ego.get("length"):
            self.ego_len = float(ego["length"])
        if ego.get("width"):
            self.ego_wid = float(ego["width"])
        vx, vy, r, delta = self._measure(exports, scene, deg, ms, rate)
        self.vx_now = vx
        if ADAPT:
            self._adapt(vx, vy, r, delta, t)
        el = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        # 执行器状态：上一帧的指令过一阶滞后
        self.ax_est += (self.ax_cmd - self.ax_est) * (1.0 - math.exp(-el / TAU_AX)) if el > 0 else 0.0

        # ---- 参考线
        width = float(lane.get("width") or 3.5)
        pts = lane.get("center_rel")
        pts = [(p[0], p[1]) for p in pts] if pts and len(pts) >= 2 else None
        pts, event = self.mem.update(pts, self._pose(exports, scene, deg), width)
        if event in (1, -1):
            self.lane_idx += event
            self.lane_changes += 1
            self.dwell_until = t + DWELL
            print("t = %.1f s：进入%s侧车道（离出发车道 %+d）" % (t, "左" if event > 0 else "右", self.lane_idx))
        if event == "hold":
            self._say("hold", "t = %.1f s：车道读数跳变（路口里常见）或暂时没有，沿记住的路径行驶" % t)
        if pts is None:
            self._say("nolane", "t = %.1f s：没有车道信息：制动停车" % t)
            self.ax_cmd = max(AX_MIN, -3.0)
            self.dc *= 0.95
            return self._output(exports, dt, deg, ms)
        a = self.veh.a
        horizon = max(vx, TARGET_KMH / 3.6) * N_STEPS * DT_MPC + 40.0
        try:
            path = Path([(x + a, y) for x, y in pts], horizon)
        except ValueError:
            self.ax_cmd = -3.0
            return self._output(exports, dt, deg, ms)
        x0 = np.zeros(NX)
        x0[S], x0[EY], x0[EPSI] = 0.0, path.ey0, path.epsi0
        x0[VX], x0[VY], x0[R], x0[DELTA], x0[AX], x0[DC] = max(vx, 0.0), vy, r, delta, self.ax_est, self.dc

        # ---- 障碍物、红绿灯、限速、车道边界
        pose = self._pose(exports, scene, deg)
        yaw = pose[2] if pose else 0.0
        v_ref_pt = (vx, vy + a * r)
        obs = obstacles_frenet(scene.get("objects"), path, a, yaw, ms, deg, v_ref_pt)
        obs = [o for o in obs if -50.0 < o.s < horizon and abs(o.d) < 10.0]
        # 本车道里在车后的（后车）不管：保持距离是它的责任（RSS 也这么规定）；自车为它刹车只会更糟。
        # 旁边车道里车后的要管：换道时不能切到它前面。
        obs = [o for o in obs if not (o.s + o.half_s < -self.ego_len / 2.0 and abs(o.d) < width / 2.0)]
        obs.sort(key=lambda o: abs(o.s) + 2.0 * abs(o.d - x0[EY]))
        if len(obs) > MAX_OBS:
            self._say("many", "t = %.1f s：障碍物多于 %d 个，只考虑最近的 %d 个" % (t, MAX_OBS, MAX_OBS))
        stop_s = self._stop_line(lane, vx)
        if stop_s is not None and not self.stopping_for:
            print("t = %.1f s：前方 %.0f m %s灯：在停止线前停车" % (
                t, float(lane.get("light_dist") or 0.0), "红" if lane.get("light_state") == "red" else "黄"))
        elif stop_s is None and self.stopping_for:
            print("t = %.1f s：%s，继续行驶" % (t, "绿灯" if lane.get("light_state") == "green" else "前方没有要停的灯了"))
        self.stopping_for = stop_s is not None
        vs, vv = self._speed_profile(path, lane, stop_s)
        left_ok = LANE_CHANGE and lane.get("left_lane") in LANE_KINDS
        right_ok = LANE_CHANGE and lane.get("right_lane") in LANE_KINDS
        if lane.get("in_junction") or event == "hold":
            left_ok = right_ok = False             # 路口里不换道
        hw = width / 2.0
        # 拓扑类：本车道 / 换到左边 / 换到右边 / 让行（本车道里减速、停下）
        hyps = [{"key": ("lane", self.lane_idx), "target": 0.0, "left": hw, "right": -hw, "vscale": 1.0}]
        if left_ok:
            hyps.append({"key": ("lane", self.lane_idx + 1), "target": width, "left": hw + width, "right": -hw,
                         "vscale": 1.0})
        if right_ok:
            hyps.append({"key": ("lane", self.lane_idx - 1), "target": -width, "left": hw, "right": -hw - width,
                         "vscale": 1.0})
        hyps.append({"key": ("yield", self.lane_idx), "target": 0.0, "left": hw, "right": -hw, "vscale": 0.0})
        self.x0 = x0
        prob = _Problem(self, path, hyps, vs, vv, obs, stop_s)
        self.solver.p = prob

        # ---- 热启动
        H, nc = len(hyps), prob.nc
        U = np.empty((H, N_STEPS, NU))
        lam = np.zeros((H, N_STEPS, nc))
        mu = np.full((H, N_STEPS, nc), self.solver.mu0)
        lamT = np.zeros((H, nc - 6))
        muT = np.full((H, nc - 6), self.solver.mu0)
        reg = np.full(H, self.solver.reg0)
        for h, hp in enumerate(hyps):
            w = self.warm.get(hp["key"])
            if w is not None and w["lam"].shape[-1] == nc:
                U[h] = self._shift(w["U"], el)
                lam[h] = self._shift(w["lam"], el)
                mu[h] = w["mu"]
                lamT[h], muT[h], reg[h] = w["lamT"], w["muT"], w["reg"]
            else:
                U[h] = self._initial_guess(x0, hp["target"], prob, brake=hp["vscale"] == 0.0)
        t0 = time.perf_counter()
        cold = any(self.warm.get(hp["key"]) is None for hp in hyps)
        self.solver.iters = ITERS_FIRST if self.n_calls == 0 else (ITERS_COLD if cold else ITERS)
        sol = self.solver.solve(x0, U, lam, mu, lamT, muT, reg)
        spent = time.perf_counter() - t0

        # ---- 选方案：先看满足约束，再比正常参考车速下的代价（让行方案也按正常车速算，慢就贵）
        prob.nominal = True
        _, nominal, _, c_all, cT_all = self.solver.total_cost(sol["X"], sol["U"], sol["lam"], sol["mu"],
                                                              sol["lamT"], sol["muT"])
        prob.nominal = False
        # 约束违反分近、远：近处（EARLY_T 内）违反 = 这个方案走不通；远处的违反后面的周期还来得及修正，只加罚
        cpos = np.maximum(c_all[:, 1:], 0.0)
        ke = int(round(EARLY_T / DT_MPC))
        bad_e = np.maximum(cpos[:, :ke].max(axis=(1, 2)) - VIOL_TOL, 0.0)
        bad_l = np.maximum(np.maximum(cpos[:, ke:].max(axis=(1, 2)), np.maximum(cT_all, 0.0).max(axis=1)) - VIOL_TOL, 0.0)
        rank = nominal + 1e6 * bad_e + LATE_PENALTY * bad_l
        # 热启动留给下一周期。同一条走廊（本车道 / 让行）的方案互相借鉴：
        # 其中一个更好（满足约束且代价低），另一个下一周期从它出发（非凸问题里少陷在差的局部解里）
        self.warm = {}
        for h, hp in enumerate(hyps):
            k_ = hp["key"]
            self.bad_n[k_] = self.bad_n.get(k_, 0) + 1 if bad_e[h] > 0 else 0
            if self.bad_n[k_] >= RESET_FRAMES:
                # 连续走不通：乘子、罚因子已经很大，解陷住了。丢掉热启动，下一周期从头来（情况可能已经变了）
                self.bad_n[k_] = 0
                continue
            src = h
            for g, gp in enumerate(hyps):
                if gp["left"] == hp["left"] and gp["right"] == hp["right"] and rank[g] < rank[src] - 1.0:
                    src = g
            self.warm[hp["key"]] = {"U": sol["U"][src].copy(), "lam": sol["lam"][src].copy(), "mu": sol["mu"][src].copy(),
                                    "lamT": sol["lamT"][src].copy(), "muT": sol["muT"][src].copy(),
                                    "reg": sol["reg"][src]}
        # 行为层：换道开始后坚持（目标车道近处连续 ABORT_FRAMES 帧走不通才放弃）；换完一次道 DWELL 秒内不再换
        changing = self.target_idx != self.lane_idx
        if changing:
            tgt = [h for h, hp in enumerate(hyps) if hp["key"] == ("lane", self.target_idx)]
            self.abort_n = self.abort_n + 1 if (not tgt or bad_e[tgt[0]] > 0) else 0
        else:
            self.abort_n = 0
        stay_ok = any(bad_e[h] == 0 for h, hp in enumerate(hyps) if hp["key"][1] == self.lane_idx)
        # 方案本身的代价（一阶滤波；走不通的不滤，立刻生效），再加上决策相关的项
        base_sc = {}
        for h, hp in enumerate(hyps):
            k_, lane_h = hp["key"], hp["key"][1]
            v_ = rank[h] + HOME_COST * abs(lane_h) \
                + (RIGHT_PASS_COST if lane_h < self.lane_idx and lane_h < 0 else 0.0)
            if bad_e[h] == 0 and k_ in self.score_ema and self.score_ema[k_] < 1e5:
                v_ = (1 - SCORE_FILTER) * self.score_ema[k_] + SCORE_FILTER * v_
            base_sc[k_] = v_
        self.score_ema = base_sc
        score = np.empty(H)
        for h, hp in enumerate(hyps):
            lane_h = hp["key"][1]
            score[h] = base_sc[hp["key"]] \
                + (SWITCH_COST if hp["key"] != self.choice else 0.0) \
                + (COMMIT_COST if changing and lane_h != self.target_idx and self.abort_n < ABORT_FRAMES else 0.0) \
                + (1e5 if not changing and lane_h != self.lane_idx and t < self.dwell_until and stay_ok else 0.0)
        hb = int(np.argmin(score))
        self.emerg_n = self.emerg_n + 1 if bool(np.all(bad_e > 0)) else 0
        emergency = self.emerg_n >= EMERGENCY_FRAMES
        if emergency:
            # 哪个方案都躲不开：不打大方向乱躲，在本车道里全力制动（和 AEB 一样）
            hb = len(hyps) - 1
            self._say("emerg%d" % (int(t) // 2), "t = %.1f s：所有方案近处都违反约束：本车道内全力制动" % t)
        key = hyps[hb]["key"]
        if key[1] != self.target_idx:
            if key[1] != self.lane_idx:
                print("t = %.1f s：决定换到%s侧车道（方案代价 %s）" % (
                    t, "左" if key[1] > self.lane_idx else "右", self._scores(hyps, score)))
            else:
                print("t = %.1f s：取消换道，留在本车道（方案代价 %s）" % (t, self._scores(hyps, score)))
        if key[0] == "yield" and not emergency and (self.choice is None or self.choice[0] != "yield"):
            print("t = %.1f s：让行：在本车道里减速%s（方案代价 %s）" % (
                t, "、停车" if stop_s is None else "", self._scores(hyps, score)))
        self.choice = key
        self.target_idx = key[1]
        u0 = sol["U"][hb, 0]
        ax_cmd = float(np.clip(u0[0], AX_MIN, AX_MAX))
        rate_cmd = float(np.clip(u0[1], -DELTA_RATE_MAX, DELTA_RATE_MAX))
        if emergency:
            ax_cmd = AX_MIN

        # ---- RSS 安全层
        rss = self._rss(obs, x0, vx, sol["X"][hb])
        if rss is not None and ax_cmd > -RSS_BRAKE:
            ax_cmd = -RSS_BRAKE
            self.rss_frames += 1
            self._say("rss%d" % (int(t) // 5), "t = %.1f s：RSS 安全距离不足（间距 %.1f m < %.1f m），计划的轨迹 1 s 内"
                      "也躲不开：强制制动" % (t, rss[0], rss[1]))
        self.ax_cmd = ax_cmd
        if vx < STANDSTILL_STEER_MS:
            rate_cmd = 0.0          # 停着：转方向盘不改变横向位置，优化器对转角“无所谓”，不让它漂
        self.dc = float(np.clip(self.dc + rate_cmd * dt, -DELTA_MAX, DELTA_MAX))

        # ---- 统计、调试、画线
        self.n_calls += 1
        self.solve_s += spent
        self.solve_max = max(self.solve_max, spent)
        off = lane.get("offset")
        if off is not None and not lane.get("in_junction"):
            self.offsets.append(abs(float(off)))
        self.debug = {"方案（目标车道）": float(key[1]), "让行": 1.0 if key[0] == "yield" else 0.0, "当前车道": float(self.lane_idx),
                      "代价": float(sol["base"][hb]), "约束违反": float(sol["viol"][hb]),
                      "求解耗时 (ms)": spent * 1000.0, "ax 指令 (m/s²)": ax_cmd, "前轮转角指令 (rad)": self.dc,
                      "前轮胎比例因子": self.veh.front_scale, "后轮胎比例因子": self.veh.rear_scale,
                      "RSS 介入": 1.0 if rss is not None else 0.0, "紧急制动": 1.0 if emergency else 0.0, "障碍物数": float(len(obs))}
        if DRAW:
            self.draw = self._lines(prob, sol, hb, path, obs)
        if t >= self.next_print:
            self.next_print = t + PRINT_EVERY
            print("t = %.1f s  车速 %.1f km/h  车道 %+d → %+d  横向偏差 %s  ax %+.2f  前轮 %+.3f rad  "
                  "违反 %.3f  求解 %.0f ms  轮胎 %.2f/%.2f" % (
                      t, vx * 3.6, self.lane_idx, key[1], "%.2f m" % off if off is not None else "—", ax_cmd,
                      self.dc, sol["viol"][hb], spent * 1000, self.veh.front_scale, self.veh.rear_scale))
        return self._output(exports, dt, deg, ms)

    def _scores(self, hyps, score):
        names = []
        for hp, sc in zip(hyps, score):
            k = hp["key"]
            n = "让行" if k[0] == "yield" else ("本车道" if k[1] == self.lane_idx else
                                               ("左" if k[1] > self.lane_idx else "右"))
            names.append("%s %s" % (n, "%.0f" % sc if sc < 1e5 else "不可行"))
        return "、".join(names)

    def _rss(self, obs, x0, vx, plan):
        """RSS：前方横向重叠的最近目标，纵向安全距离不够、而且计划的轨迹 RSS_PLAN_T 秒内
        横向也离不开它时，返回 (间距, 安全距离)；否则 None。"""
        lead, gap = None, None
        for o in obs:
            if o.s <= 0 or abs(o.d - x0[EY]) > o.half_d + self.ego_wid / 2.0 + 0.2:
                continue
            g = o.s - o.half_s - self.ego_len / 2.0
            if gap is None or g < gap:
                lead, gap = o, g
        if lead is None:
            return None
        self.min_gap = gap if self.min_gap is None else min(self.min_gap, gap)
        vf = max(lead.vs, 0.0)
        vr = max(vx, 0.0)
        d = vr * RSS_REACTION + 0.5 * RSS_ACCEL * RSS_REACTION ** 2 \
            + (vr + RSS_REACTION * RSS_ACCEL) ** 2 / (2 * RSS_BRAKE) - vf ** 2 / (2 * RSS_LEAD_BRAKE)
        d = max(d, 0.0)
        if gap < d and vr > vf + 0.1:
            n = int(round(RSS_PLAN_T / DT_MPC))
            band = lead.half_d + self.ego_wid / 2.0 + 0.2
            if all(abs(plan[k, EY] - (lead.d + lead.vd * k * DT_MPC)) <= band for k in range(n + 1)):
                return gap, d
        return None

    def _output(self, exports, dt, deg, ms):
        if OUTPUT != "carsim":
            # 加速度直接变成驱动力矩的被控对象（Chrono 宝马替身）没有刹车：停住以后再给负的 ax 车会倒着走。
            # 车速低于 STANDSTILL_MS 时负的 ax 按车速收小，停住后保持不动（往后溜了就往前推回来）
            v = self.vx_now
            ax = self.ax_cmd
            if ax < 0.0 and v < STANDSTILL_MS:
                ax = max(ax, -HOLD_GAIN * v) if v > 0.0 else -HOLD_GAIN * v
            return [ax, self.dc]
        ax, delta = self.ax_cmd, self.dc
        try:
            sw = exports["Steer_SW"] * deg
            wheel = 0.5 * (exports["Steer_L1"] + exports["Steer_R1"]) * deg
            if abs(wheel) > 0.5 and sw * wheel > 0 and 5.0 < sw / wheel < 40.0:
                self.ratio += 0.02 * (sw / wheel - self.ratio)
        except KeyError:
            pass
        v = float(exports.get("Vx", 0.0)) * ms
        if self.v_target is None:
            self.v_target = v
        self.v_target = min(v + V_TARGET_SLACK, max(v - V_TARGET_SLACK, self.v_target + ax * dt))
        err = (self.v_target - v) * 3.6
        u = ACCEL_FF * ax + KP * err + KI * self.integral
        if 0.0 < u < 1.0 or (u >= 1.0 and err < 0) or (u <= 0.0 and err > 0):
            self.integral = max(-100.0, min(100.0, self.integral + err * dt))
        return [max(0.0, min(1.0, u)), max(0.0, min(1.0, -u)) * BRAKE_MAX, math.degrees(delta) * self.ratio]

    def _lines(self, prob, sol, hb, path, obs):
        a = self.veh.a
        lines = []
        s_ref = np.arange(0.0, min(path.s[-1], 60.0), 2.0)
        x, y = path.to_xy(s_ref, np.zeros_like(s_ref))
        lines.append({"points": [[float(px - a), float(py)] for px, py in zip(x, y)], "color": [40, 120, 255], "width": 0.04})
        order = [h for h in range(prob.H) if h != hb] + [hb]
        for h in order:
            X = sol["X"][h]
            x, y = path.to_xy(X[:, S], X[:, EY])
            pts = [[float(px - a), float(py)] for px, py in zip(x[::2], y[::2])]
            if h == hb:
                col, wd = [40, 230, 60], 0.12
            elif sol["viol"][h] > 0.05:
                col, wd = [230, 40, 40], 0.05
            else:
                col, wd = [255, 160, 30], 0.05
            lines.append({"points": pts, "color": col, "width": wd})
        for j, o in enumerate(obs[:MAX_OBS]):
            s_e, d_e = prob.os[-1, j], prob.od[-1, j]
            if abs(s_e - o.s) < 0.5 and abs(d_e - o.d) < 0.5:
                continue
            x, y = path.to_xy(np.array([o.s, s_e]), np.array([o.d, d_e]))
            lines.append({"points": [[float(x[0] - a), float(y[0])], [float(x[1] - a), float(y[1])]],
                          "color": [240, 240, 240], "width": 0.05})
        return lines

    def finish(self, reason):
        if not self.n_calls:
            return
        offs = np.array(self.offsets) if self.offsets else np.zeros(1)
        print("多拓扑 NMPC 结束（%s）：车道中心偏差 均方根 %.2f m（不含路口、换道中也算）、最大 %.2f m；换道 %d 次；"
              "前方最小间距 %s；RSS 介入 %d 帧；求解平均 %.0f ms、最长 %.0f ms；轮胎比例因子 前 %.3f 后 %.3f" % (
                  reason, float(np.sqrt(np.mean(offs ** 2))), float(np.max(offs)), self.lane_changes,
                  "%.1f m" % self.min_gap if self.min_gap is not None else "—", self.rss_frames,
                  1000 * self.solve_s / self.n_calls, 1000 * self.solve_max,
                  self.veh.front_scale, self.veh.rear_scale))
