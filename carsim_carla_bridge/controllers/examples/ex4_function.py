"""例 4：入口是函数，不是类。界面“入口”填 control。"""

state = {"integral": 0.0}      # 模块级变量：每次运行都会重新加载文件，所以每次运行都从 0 开始


def control(exports, t, dt):   # 只写 3 个参数：不需要 scene
    err = 30.0 - exports["Vx"]
    state["integral"] = min(max(state["integral"] + err * dt, -100.0), 100.0)
    u = 0.08 * err + 0.02 * state["integral"]
    return [min(max(u, 0.0), 1.0), min(max(-u, 0.0), 1.0) * 8.0, 0.0]


def finish(reason):            # 可选：运行结束时调用一次
    print("运行结束：", reason)
