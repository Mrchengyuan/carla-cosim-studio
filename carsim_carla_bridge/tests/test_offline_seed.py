"""Traffic seed: everything random in the background traffic follows it, and a
run starts with the traffic lights at the start of their cycle (no CARLA server
needed: a fake world stands in for it)."""

import os
import random
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import carla  # noqa: E402
import backend_server as bs  # noqa: E402
from backend_server import Backend  # noqa: E402


class FakeBlueprint:
    def __init__(self, bp_id, colors=()):
        self.id = bp_id
        self.colors = list(colors)
        self.attrs = {}

    def has_attribute(self, key):
        return (key == "base_type" and self.id.startswith("vehicle.")) or (key == "color" and bool(self.colors)) \
            or (key == "is_invincible" and self.id.startswith("walker."))

    def get_attribute(self, key):
        if key == "base_type":
            return SimpleNamespace(as_str=lambda: "car")
        return SimpleNamespace(recommended_values=self.colors)

    def set_attribute(self, key, value):
        self.attrs[key] = value


class FakeLibrary:
    def __init__(self):
        self.bps = [FakeBlueprint("vehicle.a", ["1,1,1", "2,2,2", "3,3,3"]),
                    FakeBlueprint("vehicle.b", ["4,4,4", "5,5,5"]),
                    FakeBlueprint("walker.pedestrian.0001"), FakeBlueprint("walker.pedestrian.0002"),
                    FakeBlueprint("controller.ai.walker")]

    def filter(self, pattern):
        return [b for b in self.bps if b.id.startswith(pattern.rstrip("*"))]

    def find(self, bp_id):
        return next(b for b in self.bps if b.id == bp_id)


class FakeActor:
    next_id = 100

    def __init__(self, world, bp, tf):
        FakeActor.next_id += 1
        self.id = FakeActor.next_id
        self.world = world
        self.type_id = bp.id
        self.attributes = dict(bp.attrs)
        self.tf = tf
        self.is_alive = True
        self.calls = []

    def get_location(self):
        return self.tf.location

    def set_autopilot(self, on, port):
        self.world.log.append(("autopilot", self.id))
        self.calls.append(("autopilot", on, port))

    def start(self):
        self.calls.append(("start",))

    def go_to_location(self, loc):
        self.calls.append(("go", round(loc.x, 3), round(loc.y, 3)))

    def set_max_speed(self, speed):
        self.calls.append(("speed", speed))

    def destroy(self):
        self.is_alive = False


class FakeWorld:
    """Pedestrian navigation draws from a generator that set_pedestrians_seed
    seeds (CARLA: srand); unseeded it differs from one world to the next."""

    def __init__(self, n_points=30):
        self.lib = FakeLibrary()
        self.points = [carla.Transform(carla.Location(x=50.0 * i, y=7.0 * (i % 3), z=0.5)) for i in range(n_points)]
        self.nav = random.Random()
        self.log = []
        self.spawned = []
        self.sync = True

    def get_blueprint_library(self):
        return self.lib

    def get_map(self):
        return SimpleNamespace(get_spawn_points=lambda: list(self.points))

    def set_pedestrians_seed(self, seed):
        self.log.append(("ped_seed", seed))
        self.nav = random.Random(seed)

    def get_random_location_from_navigation(self):
        self.log.append(("nav",))
        return carla.Location(x=self.nav.uniform(-100, 100), y=self.nav.uniform(-100, 100), z=0.0)

    def try_spawn_actor(self, bp, tf, parent=None):
        a = FakeActor(self, bp, tf)
        self.spawned.append(a)
        return a

    def get_settings(self):
        return SimpleNamespace(synchronous_mode=self.sync, fixed_delta_seconds=None)

    def apply_settings(self, s):
        self.log.append(("settings", s.synchronous_mode))

    def reset_all_traffic_lights(self):
        self.log.append(("reset_lights",))

    def tick(self):
        self.log.append(("tick",))


class FakeTM:
    def __init__(self, log):
        self.log = log
        self.seeds = []

    def get_port(self):
        return 8000

    def set_random_device_seed(self, value):
        self.log.append(("tm_seed", value))
        self.seeds.append(value)

    def set_synchronous_mode(self, on):
        pass


def backend_with(world):
    b = Backend()
    b.world = world
    b.tm = FakeTM(world.log)
    b.client = SimpleNamespace(apply_batch_sync=lambda cmds, tick: None)
    return b


def spawn(seed, **kw):
    w = FakeWorld()
    b = backend_with(w)
    random.seed()  # the global generator must not matter
    b.cmd_spawn_traffic(vehicles=kw.get("vehicles", 6), walkers=kw.get("walkers", 5), seed=seed)
    return w, b


def placement(w):
    """Every actor the spawn put into the world: type, colour, place, and what
    it was told (autopilot; walker controllers: destination and speed)."""
    return [(a.type_id, a.attributes.get("color"), round(a.tf.location.x, 3), round(a.tf.location.y, 3),
             tuple(a.calls)) for a in w.spawned]


class TrafficSeedTests(unittest.TestCase):
    def test_same_seed_same_traffic(self):
        a, b, c = placement(spawn(3)[0]), placement(spawn(3)[0]), placement(spawn(4)[0])
        self.assertEqual(len(a), 6 + 5 * 2)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)
        walkers = lambda p: [x for x in p if x[0].startswith("walker.")]
        self.assertNotEqual(walkers(a), walkers(c))

    def test_seeds_are_set_before_anything_random_is_drawn(self):
        w, b = spawn(3)
        self.assertEqual(b.tm.seeds, [3])
        self.assertEqual(b.traffic_seed, 3)
        log = w.log
        self.assertLess(log.index(("tm_seed", 3)), next(i for i, e in enumerate(log) if e[0] == "autopilot"))
        self.assertLess(log.index(("ped_seed", 3)), log.index(("nav",)))

    def test_seed_is_mapped_into_carla_range(self):
        for seed, expect in ((-1, 2 ** 32 - 1), (2 ** 40 + 5, 5), (0, 0)):
            with self.subTest(seed=seed):
                w, b = spawn(seed, vehicles=1, walkers=1)
                self.assertEqual(b.tm.seeds, [expect])
                self.assertIn(("ped_seed", expect), w.log)

    def test_no_traffic_light_reset_in_the_middle_of_a_run(self):
        w = FakeWorld()
        b = backend_with(w)
        b.session, b.cosim_state = object(), "running"
        b.cmd_spawn_traffic(vehicles=2, walkers=2, seed=3)
        # Setting the traffic manager's seed resets every traffic light: not now.
        self.assertEqual(b.tm.seeds, [])
        self.assertIn(("ped_seed", 3), w.log)
        self.assertNotIn(("tick",), w.log)  # only the run ticks
        self.assertEqual(len(b._pending_walkers), 2)

    def test_cars_moved_off_the_ego_spawn_point_follow_the_seed(self):
        def moved(seed, global_seed):
            w = FakeWorld()
            b = backend_with(w)
            b.traffic_seed = seed
            anchor = w.points[0]
            b.traffic["vehicles"] = [FakeActor(w, w.lib.find("vehicle.a"), anchor),
                                     FakeActor(w, w.lib.find("vehicle.b"), anchor)]
            random.seed(global_seed)
            self.assertTrue(b._clear_spawn_point(anchor, 0))
            return [(a.type_id, round(a.tf.location.x, 3)) for a in b.traffic["vehicles"]]

        self.assertEqual(len(moved(3, 1)), 2)
        self.assertEqual(moved(3, 1), moved(3, 2))
        self.assertNotEqual(moved(3, 1), moved(4, 1))


class RunStartTests(unittest.TestCase):
    def test_run_start_resets_traffic_lights_before_settling(self):
        w = FakeWorld()
        w.sync = False
        b = backend_with(w)
        ego = SimpleNamespace(id=1, is_alive=True, type_id="vehicle.test", attributes={})
        b.cmd_spawn_ego = lambda *args: setattr(b, "ego", ego)

        class Session:
            scene = None
            exports = None

            def __init__(self, *args):
                pass

            def start(self):
                return {"external_api": False, "reference_point": [0, 0, 0], "inner_steps": 1,
                        "clock_warning": False}

        try:
            with mock.patch.object(bs, "CoSimSession", Session), mock.patch.object(bs, "CarlaDriveSession", Session), \
                    mock.patch.object(bs.rigmod, "spec_of", return_value={}):
                b.cmd_cosim_start({"carsim": {"mock": True}, "run": {"log_path": ""}})
            log = w.log
            self.assertEqual(log.count(("reset_lights",)), 1)
            self.assertLess(log.index(("settings", True)), log.index(("reset_lights",)))
            self.assertLess(log.index(("reset_lights",)), log.index(("tick",)))
            self.assertEqual(b.cosim_state, "running")
        finally:
            b.session, b.cosim_state = None, "stopped"


if __name__ == "__main__":
    unittest.main()
