"""批量测试 report: the runs of a batch (the GUI runs them one after the
other, each a normal run whose record is a folder in the batch's dir), their
key figures from run.json in one table, pass / fail: the run's verdict (通过标准,
criteria.py, run.json "verdict"; runs from before it: finished, no collision, never off
the driving lanes). Written as report.csv
and report.md in the batch's dir (relative: the bridge dir). Files only.
"""
import csv
import json
import os
import time

COLUMNS = [("lane_offset_rms", "车道偏差均方根 m", "%.3f"), ("lane_offset_max", "车道偏差最大 m", "%.3f"),
           ("heading_err_max", "最大航向偏差", "%.2f"), ("time_off_lane", "出车道 s", "%.2f"),
           ("collisions", "碰撞次数", "%d"), ("min_gap_ahead", "前方最小间距 m", "%.2f"), ("ttc_min", "最小 TTC s", "%.2f"),
           ("decel_max", "最大减速度 m/s²", "%.2f"), ("jerk_max", "最大 jerk m/s³", "%.1f"), ("distance", "行驶距离 m", "%.0f")]


def _fmt(v, f):
    if v is None:
        return ""
    try:
        return f % v
    except (TypeError, ValueError):
        return str(v)


def summarize(out_dir, items, base=None):
    """items: [{"label", "record_dir" ("" when it did not start), "state", "detail"}]. Returns
    {"rows": [{"label", "state", "detail", "passed", "record_dir", kpi keys...}], "csv", "md", "passed", "total"}."""
    base = os.path.abspath(base or os.getcwd())
    out_dir = os.path.abspath(os.path.join(base, out_dir))
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    for it in items:
        rd = it.get("record_dir") or ""
        run = {}
        if rd and os.path.isfile(os.path.join(rd, "run.json")):
            try:
                with open(os.path.join(rd, "run.json"), encoding="utf-8") as f:
                    run = json.load(f)
            except (OSError, ValueError):
                run = {}
        k = run.get("kpi") or {}
        state = it.get("state") or run.get("end") or ""
        finished = state == "finished"
        v = run.get("verdict")
        if isinstance(v, dict):
            passed, fails = bool(v.get("passed")), list(v.get("fails") or [])
        else:  # a run from before 通过标准
            passed = bool(finished and (k.get("collisions") or 0) == 0 and (k.get("time_off_lane") or 0.0) == 0.0 and k)
            fails = [] if passed else ["没有跑完、碰撞或开出车道"]
        row = {"label": it.get("label", ""), "state": state, "detail": it.get("detail") or run.get("end_reason") or "",
               "passed": passed, "fails": fails, "record_dir": rd}
        row.update({key: k.get(key) for key, _, _ in COLUMNS})
        rows.append(row)
    csv_path = os.path.join(out_dir, "report.csv")
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:  # -sig: Excel reads the Chinese headers
        w = csv.writer(f)
        w.writerow(["测试项", "结果", "未通过的原因", "结束"] + [name for _, name, _ in COLUMNS] + ["说明", "运行记录"])
        for r in rows:
            w.writerow([r["label"], "通过" if r["passed"] else "未通过", "；".join(r["fails"]), r["state"]]
                       + [_fmt(r[k], fm) for k, _, fm in COLUMNS] + [r["detail"], r["record_dir"]])
    md_path = os.path.join(out_dir, "report.md")
    n_pass = sum(1 for r in rows if r["passed"])
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# 批量测试报告\n\n%s，共 %d 项，通过 %d 项（通过标准见“批量测试”页；未通过的原因写在“说明”里）\n\n"
                % (time.strftime("%Y-%m-%d %H:%M:%S"), len(rows), n_pass))
        f.write("| 测试项 | 结果 | " + " | ".join(name for _, name, _ in COLUMNS) + " | 说明 |\n")
        f.write("|---|---|" + "---|" * len(COLUMNS) + "---|\n")
        for r in rows:
            f.write("| %s | %s | %s | %s |\n" % (r["label"], "✅ 通过" if r["passed"] else "❌ 未通过",
                                                 " | ".join(_fmt(r[k], fm) for k, _, fm in COLUMNS),
                                                 "；".join(r["fails"] + [r["detail"]]).replace("|", "/")))
    return {"rows": rows, "csv": csv_path, "md": md_path, "passed": n_pass, "total": len(rows), "dir": out_dir}
