"""Connection and run state left over from crashes, reconnects, map changes and
exits, without a CARLA server: the trash button on the 场景对象 page, a
reconnect to a CARLA that died unnoticed, Windows finding a CARLA that exited,
a failed map change, pedestrians spawned while paused, an idle-ticked world at
exit, the run state of a new connection, vehicle_specs with an unknown model,
Ctrl+C, a second GUI, dataset commands beside the worker."""

import json
import os
import queue
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import carla  # noqa: E402
import backend_server as bs  # noqa: E402
from backend_server import Backend  # noqa: E402


class FakeActor:
    def __init__(self, id, type_id, parent=None, log=None):
        self.id, self.type_id, self.parent = id, type_id, parent
        self.attributes = {}
        self.is_alive = True
        self.log = log if log is not None else []

    def destroy(self):
        self.log.append(("destroy", self.id))
        self.is_alive = False
        return True

    def stop(self):
        self.log.append(("stop", self.id))


class ActorList(list):
    def filter(self, pattern):
        return ActorList(a for a in self if a.type_id.startswith(pattern.rstrip("*")))


class FakeWorld:
    def __init__(self, actors=(), sync=False):
        self.actors = ActorList(actors)
        self.settings = SimpleNamespace(synchronous_mode=sync, fixed_delta_seconds=0.05 if sync else None)
        self.applied = []

    def get_actor(self, i):
        return next((a for a in self.actors if a.id == i), None)

    def get_actors(self):
        return self.actors

    def get_settings(self):
        return SimpleNamespace(**vars(self.settings))

    def apply_settings(self, s):
        self.applied.append(s)
        self.settings = s

    def wait_for_tick(self, seconds=None):
        pass

    def tick(self):
        pass


class DestroyActorTests(unittest.TestCase):
    def setUp(self):
        self.log = []
        self.b = Backend()
        self.ego = FakeActor(1, "vehicle.tesla.model3", log=self.log)
        self.cam = FakeActor(2, "sensor.camera.rgb", parent=self.ego, log=self.log)
        self.car = FakeActor(3, "vehicle.audi.tt", log=self.log)
        self.walker = FakeActor(4, "walker.pedestrian.0001", log=self.log)
        self.ctrl = FakeActor(5, "controller.ai.walker", parent=self.walker, log=self.log)
        self.loose = FakeActor(6, "sensor.other.collision", log=self.log)  # on no car of ours
        self.b.world = FakeWorld([self.ego, self.cam, self.car, self.walker, self.ctrl, self.loose])
        self.b.ego = self.ego
        self.b.traffic = {"vehicles": [self.car], "walkers": [self.walker], "controllers": [self.ctrl]}
        self.b._pending_walkers = [(self.ctrl, 1.2)]

    def test_sensor_on_the_ego_is_refused(self):
        # A live view or a run's rig sensor: the run would wait 5 s a frame for its data.
        with self.assertRaisesRegex(RuntimeError, "实时画面"):
            self.b.cmd_destroy_actor(2)
        self.assertEqual(self.log, [])

    def test_traffic_car_leaves_the_bookkeeping(self):
        # The traffic left, for the count on the 交通流 page.
        self.assertEqual(self.b.cmd_destroy_actor(3), {"vehicles": 0, "walkers": 1})
        self.assertEqual(self.log, [("destroy", 3)])
        self.assertEqual(self.b.traffic["vehicles"], [])  # never destroyed again by id later

    def test_pedestrian_goes_with_its_controller(self):
        self.assertEqual(self.b.cmd_destroy_actor(4), {"vehicles": 1, "walkers": 0})
        self.assertEqual(self.log, [("stop", 5), ("destroy", 5), ("destroy", 4)])
        self.assertEqual(self.b.traffic, {"vehicles": [self.car], "walkers": [], "controllers": []})
        self.assertEqual(self.b._pending_walkers, [])

    def test_other_actors_are_deleted(self):
        self.assertIs(self.b.cmd_destroy_actor(6), True)  # not traffic: no count for the GUI
        self.assertEqual(self.log, [("destroy", 6)])


class ReconnectTests(unittest.TestCase):
    def client_class(self, calls, world):
        class Client:
            def __init__(self, host, port):
                calls.append(("new client", host, port))

            def set_timeout(self, t):
                calls.append(("new timeout", t))

            def get_world(self):
                return world

            def get_server_version(self):
                return "0.9.16"

            def get_client_version(self):
                return "0.9.16"
        return Client

    def backend(self, calls):
        b = Backend()
        b.world = FakeWorld()
        b.client = SimpleNamespace(set_timeout=lambda t: calls.append(("old timeout", t)))
        b.carla_addr = ("localhost", 2000)
        b.cmd_world_info = lambda: {"map": "Town10HD_Opt", "cosim_state": b.cosim_state}
        b._teardown = lambda: calls.append("teardown")
        b._carla_listening = lambda now=False, record=True: True  # the new CARLA
        return b

    def test_dead_carla_is_let_go_without_cleanup_calls(self):
        calls, new_world = [], FakeWorld()
        b = self.backend(calls)
        b.traffic["vehicles"] = [FakeActor(7, "vehicle.audi.tt")]
        b.ego = FakeActor(8, "vehicle.tesla.model3")
        b._pending_walkers = [(FakeActor(9, "controller.ai.walker"), 1.0)]
        b.cosim_state, b.cosim_detail = "error", "CARLA 服务器已退出"
        b._carla_listening_now = lambda: False  # the old CARLA's port: another process, or nobody
        with mock.patch.object(bs.carla, "Client", self.client_class(calls, new_world)):
            info = b.cmd_connect("localhost", 2000)
        self.assertNotIn("teardown", calls)
        self.assertEqual(b.traffic, {"vehicles": [], "walkers": [], "controllers": []})
        self.assertEqual(b._pending_walkers, [])
        self.assertIsNone(b.ego)
        self.assertIs(b.world, new_world)
        self.assertEqual(info["cosim_state"], "stopped")  # not the old run's "error"
        self.assertEqual(b.cosim_detail, "")

    def test_live_carla_is_cleaned_up_with_a_short_timeout(self):
        calls = []
        b = self.backend(calls)
        b._carla_listening_now = lambda: None  # another host: can't tell
        with mock.patch.object(bs.carla, "Client", self.client_class(calls, FakeWorld())):
            b.cmd_connect("10.0.0.2", 2000)
        i = calls.index("teardown")
        self.assertEqual(calls[i - 1:i + 2], [("old timeout", 3.0), "teardown", ("old timeout", 20.0)])

    def test_clear_traffic_forgets_ids_even_if_carla_does_not_answer(self):
        b = Backend()
        w = FakeWorld()

        def timeout():
            raise RuntimeError("time-out of 20000ms while waiting for the simulator")
        w.get_settings = timeout
        b.world = w
        b.client = SimpleNamespace(apply_batch_sync=lambda *a: None)
        ctrl = FakeActor(11, "controller.ai.walker")
        b.traffic = {"vehicles": [FakeActor(10, "vehicle.audi.tt")], "walkers": [], "controllers": [ctrl]}
        b._pending_walkers = [(ctrl, 1.0)]
        with self.assertRaisesRegex(RuntimeError, "time-out"):
            b.cmd_clear_traffic()
        self.assertEqual(b.traffic, {"vehicles": [], "walkers": [], "controllers": []})
        self.assertEqual(b._pending_walkers, [])


class CarlaGoneTests(unittest.TestCase):
    def backend(self, host="localhost"):
        b = Backend()
        b.world = FakeWorld()
        b.carla_addr = (host, 2000)
        return b

    def test_windows_idle_heartbeat_looks_with_netstat_and_the_worker_checks_at_once(self):
        b = self.backend()
        looks, lost = [], []
        b._carla_listening = lambda now=False, record=True: looks.append(now) or False
        b._forget_carla = lost.append
        with mock.patch.object(bs.os, "name", "nt"):
            last = b.heartbeat_step(0.0)
            self.assertEqual(b.heartbeat_step(last), last)  # every 5 s, not every second
            self.assertEqual(looks, [True])
            self.assertTrue(b._gone_hint)
            b._alive_check = time.time()  # the worker's own 2 s check would wait
            b._check_carla()
        self.assertEqual(looks, [True, True])
        self.assertEqual(len(lost), 1)
        self.assertFalse(b._gone_hint)

    def test_windows_busy_heartbeat_reports_carla_gone(self):
        b = self.backend()
        events, timeouts = [], []
        b.emit = events.append
        b.client = SimpleNamespace(set_timeout=timeouts.append)
        b.task = ("load_map", time.time() - 3.0)
        b._carla_listening = lambda now=False, record=True: False if now else None  # only netstat can tell
        with mock.patch.object(bs.os, "name", "nt"):
            b.heartbeat_step()
        self.assertTrue(events[0]["carla_gone"])
        self.assertEqual(timeouts, [0.5])
        self.assertTrue(b._fast_timeout)

    def test_another_host_is_never_taken_for_gone_by_the_heartbeat(self):
        b = self.backend("10.0.0.2")
        with mock.patch.object(bs.os, "name", "nt"):
            b.heartbeat_step(0.0)
        self.assertFalse(b._gone_hint)

    def test_windows_carla_hidden_from_netstat_is_not_taken_for_gone(self):
        # e.g. WSL2 mirrored networking: connected fine, but netstat lists no listener.
        b = self.backend()
        runs = []

        def netstat(*a, **k):
            runs.append(a)
            return SimpleNamespace(stdout="  TCP    0.0.0.0:135    0.0.0.0:0    LISTENING    900\n")
        with mock.patch.object(bs.os, "name", "nt"), mock.patch.object(bs.os.path, "exists", lambda p: False), \
                mock.patch.object(bs.subprocess, "run", netstat):
            b._proc_sees_carla = True  # as cmd_connect does before its look
            b._proc_sees_carla = b._carla_listening(now=True) is not False
            self.assertFalse(b._proc_sees_carla)
            b.heartbeat_step(0.0)
            b.task = ("load_map", time.time() - 3.0)
            b.heartbeat_step()
        self.assertEqual(len(runs), 1)  # only the look while connecting
        self.assertFalse(b._gone_hint)
        self.assertFalse(getattr(b, "_fast_timeout", False))

    def test_heartbeat_look_at_the_old_carla_during_a_reconnect_is_dropped(self):
        b = self.backend()
        b._carla_listener = {"111"}
        switch = [("localhost", 3000), None]

        def netstat(*a, **k):
            if b.carla_addr[1] == 2000:  # the worker connects to another CARLA while netstat runs
                b.carla_addr, b._carla_listener = switch
            return SimpleNamespace(stdout="  TCP    0.0.0.0:2000    0.0.0.0:0    LISTENING    111\n"
                                          "  TCP    0.0.0.0:3000    0.0.0.0:0    LISTENING    222\n")
        with mock.patch.object(bs.os, "name", "nt"), mock.patch.object(bs.os.path, "exists", lambda p: False), \
                mock.patch.object(bs.subprocess, "run", netstat):
            self.assertIsNone(b._carla_listening_now(record=False))
            self.assertIsNone(b._carla_listener)  # not the old CARLA's process kept as the new one's
            self.assertIs(b._carla_listening(now=True), True)  # the worker's look after connecting
            self.assertEqual(b._carla_listener, {"222"})
            # Idle, and the worker has recorded the new CARLA before the old port's owners are compared.
            b.carla_addr, b._carla_listener, switch[1] = ("localhost", 2000), {"111"}, {"222"}
            b.heartbeat_step(0.0)
        self.assertFalse(b._gone_hint)

    def test_lost_carla_drops_walkers_waiting_for_a_frame(self):
        b = self.backend()
        b.client = SimpleNamespace(set_timeout=lambda t: None)
        b._pending_walkers = [(FakeActor(5, "controller.ai.walker"), 1.0)]
        b._start_walkers = lambda ctrls: self.fail("started walkers of a CARLA that is gone")
        b._forget_carla("gone")
        self.assertEqual(b._pending_walkers, [])
        self.assertIsNone(b.world)


class MapChangeTests(unittest.TestCase):
    def test_failed_load_destroys_the_ego_first_and_follows_the_server(self):
        log = []
        b = Backend()
        new = FakeWorld()
        b.world, b.ego, b.anchor = FakeWorld(), FakeActor(1, "vehicle.tesla.model3", log=log), "anchor"
        timeouts = []
        b.client = SimpleNamespace(set_timeout=timeouts.append, get_world=lambda: new)

        def load():
            log.append("load")
            raise RuntimeError("failed to connect to newly created map")
        with self.assertRaisesRegex(RuntimeError, "newly created map"):
            b._switch_world(load)
        self.assertEqual(log, [("destroy", 1), "load"])  # not left behind, untracked, on its spawn point
        self.assertIs(b.world, new)  # not the expired episode
        self.assertIsNone(b.ego)
        self.assertEqual(timeouts[-1], 20.0)

    def test_map_change_clears_an_old_run_state(self):
        b = Backend()
        b.world = FakeWorld()
        b.client = SimpleNamespace(set_timeout=lambda t: None)
        b.cosim_state, b.cosim_detail = "finished", "达到设定的运行时长 10 s"
        b.cmd_world_info = lambda: {"cosim_state": b.cosim_state}
        self.assertEqual(b._switch_world(lambda: FakeWorld())["cosim_state"], "stopped")
        self.assertEqual(b.cosim_detail, "")


class RunStopTests(unittest.TestCase):
    def test_walkers_spawned_while_paused_start_when_the_run_stops(self):
        b = Backend()
        b.world = FakeWorld()
        b.session, b.cosim_state = SimpleNamespace(stop=lambda release_vehicle: None), "paused"
        ctrl = FakeActor(5, "controller.ai.walker")
        b._pending_walkers = [(ctrl, 1.3)]
        started = []
        b._start_walkers = started.append
        b._stop_cosim_if_running("stopped")
        self.assertEqual(started, [[(ctrl, 1.3)]])
        self.assertEqual(b._pending_walkers, [])
        self.assertEqual(b.cosim_state, "stopped")

    def test_idle_ticked_world_is_left_asynchronous_at_exit(self):
        b = Backend()
        w = FakeWorld(sync=True)
        b.world, b.idle_tick = w, True
        b._teardown = lambda: None
        b.cleanup()
        self.assertFalse(w.settings.synchronous_mode)
        self.assertIsNone(w.settings.fixed_delta_seconds)

    def test_sync_world_without_idle_tick_is_left_alone(self):
        b = Backend()
        w = FakeWorld(sync=True)
        b.world = w
        b._teardown = lambda: None
        b.cleanup()
        self.assertEqual(w.applied, [])


class VehicleSpecsTests(unittest.TestCase):
    def test_unknown_or_unspawnable_models_are_skipped(self):
        b = Backend()
        events = []
        b.emit = events.append

        class Probe:
            def __init__(self, tf):
                loc = tf.location
                wheel = lambda x, y: SimpleNamespace(position=carla.Vector3D((loc.x + x) * 100, (loc.y + y) * 100, loc.z * 100),
                                                     radius=37.0, max_steer_angle=70.0)
                self.pc = SimpleNamespace(wheels=[wheel(1.4, -0.8), wheel(1.4, 0.8), wheel(-1.5, -0.8), wheel(-1.5, 0.8)],
                                          mass=1845.0)
                self.bounding_box = SimpleNamespace(extent=SimpleNamespace(x=2.4, y=1.0, z=0.75), location=SimpleNamespace(z=0.75))

            def get_physics_control(self):
                return self.pc

            def set_simulate_physics(self, on):
                pass

            def destroy(self):
                pass

        class Library:
            def find(self, vid):
                if vid == "vehicle.no.such_car":
                    raise IndexError("blueprint 'vehicle.no.such_car' not found")
                return SimpleNamespace(id=vid, set_attribute=lambda k, v: None)

        b.world = SimpleNamespace(
            get_blueprint_library=Library,
            get_map=lambda: SimpleNamespace(get_spawn_points=lambda: [carla.Transform(carla.Location(10.0, 20.0, 0.3))]),
            get_actors=lambda: ActorList(),
            try_spawn_actor=lambda bp, tf: None if bp.id == "vehicle.blocked" else Probe(tf))
        specs = b.cmd_vehicle_specs(["vehicle.no.such_car", "vehicle.tesla.model3", "vehicle.blocked"])
        self.assertEqual(list(specs), ["vehicle.tesla.model3"])
        self.assertAlmostEqual(specs["vehicle.tesla.model3"]["wheelbase_m"], 2.9, places=3)
        warn = [e["msg"] for e in events if e.get("event") == "log" and e.get("level") == "warn"]
        self.assertEqual(len(warn), 1)
        self.assertIn("vehicle.no.such_car", warn[0])
        self.assertIn("vehicle.blocked", warn[0])


class ConnectionTests(unittest.TestCase):
    def test_second_gui_is_told_and_let_go(self):
        b = Backend()
        threading.Thread(target=b.run_worker, daemon=True).start()
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]
        threading.Thread(target=bs.serve_clients, args=(srv, b), daemon=True).start()

        def ask(conn, f, rid):
            conn.sendall((json.dumps({"id": rid, "cmd": "ping"}) + "\n").encode())
            return json.loads(f.readline())

        first = socket.create_connection(("127.0.0.1", port), timeout=5)
        f1 = first.makefile("rb")
        second = third = fourth = None
        try:
            self.assertEqual(ask(first, f1, 1)["result"], "pong")
            second = socket.create_connection(("127.0.0.1", port), timeout=5)
            f2 = second.makefile("rb")
            msg = json.loads(f2.readline())
            self.assertTrue(msg.get("rejected"))
            self.assertIn("另一个界面窗口", msg["msg"])
            self.assertEqual(f2.readline(), b"")  # closed: no silent wait for replies
            f2.close()
            self.assertEqual(ask(first, f1, 2)["result"], "pong")  # the first one goes on
            f1.close()
            first.close()
            reply = {}
            for _ in range(50):  # once the first has gone, the next one is served
                third = socket.create_connection(("127.0.0.1", port), timeout=5)
                f3 = third.makefile("rb")
                try:
                    reply = ask(third, f3, 3)
                except (OSError, ValueError):  # turned away before the ping went out
                    reply = {"rejected": True}
                f3.close()
                if not reply.get("rejected"):
                    break
                third.close()
                third = None
                time.sleep(0.1)
            self.assertEqual(reply.get("result"), "pong")
            # (The third one stays connected.) A backend on its way out (closed GUI opened again right away) turns
            # nobody away: the GUI is dropped when it exits and finds the new one.
            b._cleaned = True
            fourth = socket.create_connection(("127.0.0.1", port), timeout=5)
            fourth.settimeout(0.5)
            with self.assertRaises(socket.timeout):
                fourth.recv(100)
        finally:
            for s in (first, second, third, fourth, srv):
                if s is not None:
                    s.close()

    def test_file_commands_are_answered_while_the_worker_is_busy(self):
        b = Backend()
        threading.Thread(target=b.run_io_worker, daemon=True).start()
        replies = queue.Queue()
        b.submit({"id": 1, "cmd": "load_map", "args": {"name": "Town01"}}, replies.put)  # the worker is busy elsewhere
        with tempfile.TemporaryDirectory() as d:
            b.submit({"id": 2, "cmd": "disk_info", "args": {"path": d}}, replies.put)
            b.submit({"id": 3, "cmd": "dataset_list", "args": {"out_dir": d}}, replies.put)
            b.submit({"id": 4, "cmd": "dataset_delete", "args": {"root": d}}, replies.put)
            got = [replies.get(timeout=5) for _ in range(3)]
            self.assertTrue(os.path.isdir(d))  # not a dataset: refused
        self.assertEqual([(r["id"], r["ok"]) for r in got], [(2, True), (3, True), (4, False)])
        self.assertEqual(b.requests.qsize(), 1)  # the CARLA command stays with the worker
        self.assertTrue(replies.empty())

    def test_dataset_still_being_written_is_not_deleted(self):
        b = Backend()
        with tempfile.TemporaryDirectory() as d:
            for f in ("calib.json", "meta.json"):
                open(os.path.join(d, f), "w").close()
            answers = []

            class Collector:
                root = d

                def stop(self):  # the writer puts its last frames on disk
                    try:
                        b.cmd_dataset_delete(d)
                    except RuntimeError as e:
                        answers.append(str(e))
            b.collector = Collector()
            b._stop_cosim_if_running("finished")
            self.assertEqual(answers, ["这个数据集正在采集中"])
            self.assertTrue(os.path.isdir(d))
            self.assertIsNone(b.collector)
            self.assertEqual(b.cmd_dataset_delete(d)["deleted"], d)  # once it is on disk
            self.assertFalse(os.path.isdir(d))
            os.makedirs(d)  # for TemporaryDirectory's own cleanup

    @unittest.skipUnless(os.name == "posix", "SIGINT from a terminal: POSIX")
    def test_ctrl_c_takes_the_bounded_exit(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        p = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "..", "backend_server.py"), "--port", str(port)],
                             cwd=os.path.join(HERE, ".."), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        watchdog = threading.Timer(60.0, p.kill)
        watchdog.start()
        try:
            lines = []
            while not any("listening" in l for l in lines):
                line = p.stdout.readline()
                if not line:
                    break
                lines.append(line)
            self.assertTrue(any("listening" in l for l in lines), lines[-5:])
            p.send_signal(signal.SIGINT)
            out = p.communicate(timeout=30)[0]
        finally:
            watchdog.cancel()
            if p.poll() is None:
                p.kill()
                p.wait()
        self.assertIn("exit: cleanup done", out)
        self.assertNotIn("KeyboardInterrupt", out)
        self.assertEqual(p.returncode, 0)


if __name__ == "__main__":
    unittest.main()
