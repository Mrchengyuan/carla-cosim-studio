"""通过标准（config "criteria"，界面“批量测试”页的“通过标准”卡片）：每次运行结束时按它判定通过 / 未通过，
写进 run.json 的 "verdict"，输出窗口说明原因；批量测试的报告用同一个判定。

    finished      要跑完设定的时长（被停止、出错算未通过）
    no_collision  不能有碰撞
    on_lane       不能开出行车道
    limits        各项指标的上 / 下限（None = 不检查）：
                    lane_offset_rms、lane_offset_max  车道偏差 m        ≤
                    ttc_min                           最小碰撞时间 s    ≥
                    min_gap_ahead                     前方最小间距 m    ≥
                    accel_max、decel_max              最大加速 / 减速 m/s²  ≤
                    jerk_max                          最大纵向 jerk m/s³    ≤
                  这一项没有数据（例如前方没有接近的目标，TTC 为空）时不算违反。
"""

LIMITS = [  # key, name, unit, ">=" or "<="
    ("lane_offset_rms", "车道偏差均方根", "m", "<="),
    ("lane_offset_max", "车道偏差最大", "m", "<="),
    ("ttc_min", "最小碰撞时间 TTC", "s", ">="),
    ("min_gap_ahead", "前方最小间距", "m", ">="),
    ("accel_max", "最大加速度", "m/s²", "<="),
    ("decel_max", "最大减速度", "m/s²", "<="),
    ("jerk_max", "最大纵向 jerk", "m/s³", "<="),
]

DEFAULT = {"finished": True, "no_collision": True, "on_lane": True, "limits": {k: None for k, _, _, _ in LIMITS}}


def verdict(kpi, end, criteria=None):
    """{"passed", "fails": [plain words], "checked": [what was checked]} of a run: its key figures
    (RunKpi.result(), may be None), how it ended ("finished" / "stopped" / "error")."""
    c = dict(DEFAULT, **(criteria or {}))
    k = kpi or {}
    fails, checked = [], []
    if c.get("finished"):
        checked.append("跑完设定的时长")
        if end != "finished":
            fails.append({"stopped": "没有跑完（被停止）", "error": "没有跑完（出错）"}.get(end, "没有跑完（%s）" % end))
    if c.get("no_collision"):
        checked.append("没有碰撞")
        if k.get("collisions"):
            fails.append("碰撞 %d 次" % k["collisions"])
    if c.get("on_lane"):
        checked.append("不开出车道")
        if k.get("time_off_lane"):
            fails.append("开出车道 %.2f s" % k["time_off_lane"])
    for key, name, unit, op in LIMITS:
        lim = (c.get("limits") or {}).get(key)
        if lim is None:
            continue
        lim = float(lim)
        checked.append("%s %s %g %s" % (name, "≥" if op == ">=" else "≤", lim, unit))
        v = k.get(key)
        if v is None:
            continue
        if (op == ">=" and v < lim) or (op == "<=" and v > lim):
            fails.append("%s %.3g %s %s %g %s" % (name, v, unit, "<" if op == ">=" else ">", lim, unit))
    if not k and (c.get("no_collision") or c.get("on_lane")):
        fails.append("没有运行指标（没有采样）")
    return {"passed": not fails, "fails": fails, "checked": checked}


def text(v):
    """The verdict in one line for the output window."""
    return "判定：通过" if v["passed"] else "判定：未通过：" + "；".join(v["fails"])
