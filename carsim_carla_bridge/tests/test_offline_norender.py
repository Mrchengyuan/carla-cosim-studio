"""不需要 CARLA：批量测试加速和恢复用到的两件事。

关闭渲染（sync.no_render）什么时候不能关：要采集数据、算法要用相机（rgb / 深度 / 语义 / 实例；预设套件的相机按预设判断）；
只用激光雷达、毫米波雷达或不要传感器时可以关。carla_port_ready：一个端口有没有程序在监听，从系统的套接字表看
（不连接）：本机开着的端口是 True、关掉后是 False、别的主机说不知道（None）；这个命令走文件线程。

    python tests/test_offline_norender.py
"""
import os
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import backend_server  # noqa: E402
import settings  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + (("  " + str(detail)) if detail != "" else ""), flush=True)
    if not cond:
        FAILS.append(name)


def main():
    d = settings.default_dict()
    check("no sensors, no collection: rendering can be off", backend_server.no_render_blocked(d) == "")
    d["collect"]["enabled"] = True
    check("collecting: kept on, said", "采集" in backend_server.no_render_blocked(d))
    d = settings.default_dict()
    d["rig"] = {"preset": "nuscenes", "sensors": [], "frame": "carsim"}
    d["scene"]["sensors"] = ["lidar_top", "radar_front"]
    check("only a lidar and a radar for the algorithm: can be off", backend_server.no_render_blocked(d) == "",
          backend_server.no_render_blocked(d))
    d["scene"]["sensors"] = ["lidar_top", "cam_front"]
    check("a camera of the preset for the algorithm: kept on, which one said", "cam_front" in backend_server.no_render_blocked(d),
          backend_server.no_render_blocked(d))
    d = settings.default_dict()
    d["rig"]["sensors"] = [{"name": "my_depth", "type": "depth"}, {"name": "my_imu", "type": "imu"}]
    d["scene"]["sensors"] = ["my_imu"]
    r1 = backend_server.no_render_blocked(d)
    d["scene"]["sensors"] = ["my_depth"]
    r2 = backend_server.no_render_blocked(d)
    check("the config's own rig: an IMU can, a depth camera cannot", r1 == "" and "my_depth" in r2, (r1, r2))
    check("sync.no_render is off by default", settings.default_dict()["sync"]["no_render"] is False)

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    b = backend_server.Backend.__new__(backend_server.Backend)  # (the command needs no state)
    up = b.cmd_carla_port_ready("localhost", port)
    srv.close()
    down = b.cmd_carla_port_ready("localhost", port)
    check("carla_port_ready: a listening port True, closed False (from the socket table)",
          up == {"listening": True} and down == {"listening": False}, (up, down))
    check("... another host: can't tell", b.cmd_carla_port_ready("10.0.0.5", port) == {"listening": None})
    check("... on the file thread", "carla_port_ready" in backend_server.IO_CMDS)
    print("FAILED: %s" % FAILS if FAILS else "ALL NO-RENDER / PORT TESTS PASSED")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
