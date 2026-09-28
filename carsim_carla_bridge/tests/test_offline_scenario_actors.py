"""测试场景 moving actors without a CARLA server: the config checked in plain
words (type, ranges, a cut-in from the ego's own lane, a brake or cut-in
without its parameter); the motion rules on a straight path with a fake
client (spawned with physics off; a slow car's speed held; a car that
brakes once the ego is within its trigger distance, to a stop; a cut-in
over its time with a smooth lateral profile, into the ego's lane; a
pedestrian that waits, then crosses at its speed and stops at the far side;
the velocities given to the scene, in CARLA's world frame; placed with one
synchronous batch per frame).

    python tests/test_offline_scenario_actors.py
"""
import os
import sys
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import carla  # noqa: E402

import scenario  # noqa: E402


def actor(**kw):
    a = dict(type="slow_car", distance_m=50, lane=0, speed_kmh=36, trigger_m=20, param=0)
    a.update(kw)
    return a


class CheckTests(unittest.TestCase):
    def test_normalised(self):
        (a,) = scenario.check_actors([actor(lane="1", speed_kmh="36")])
        self.assertEqual((a["lane"], a["speed_kmh"]), (1, 36.0))

    def test_plain_words(self):
        for bad, words in ((actor(type="ufo"), "类型 'ufo' 不认识"), (actor(speed_kmh=500), "speed_kmh = 500 超出范围"),
                           (actor(type="cut_in", lane=0, param=2), "要在相邻车道"),
                           (actor(type="lead_brake", param=0), "参数要大于 0（减速度 m/s²）"),
                           (actor(type="cut_in", lane=1, param=0), "参数要大于 0（切入用时 s）"),
                           (actor(distance_m="far"), "distance_m 不是数字")):
            with self.assertRaises(ValueError) as cm:
                scenario.check_actors([bad])
            self.assertIn(words, str(cm.exception))
        with self.assertRaisesRegex(ValueError, "应为列表"):
            scenario.check_actors({"type": "slow_car"})


class FakeBP:
    def __init__(self):
        self.attrs = {}

    def has_attribute(self, k):
        return k == "role_name"

    def set_attribute(self, k, v):
        self.attrs[k] = v


class FakeWorld:
    def get_blueprint_library(self):
        class Lib:
            def find(self, model):
                bp = FakeBP()
                bp.model = model
                return bp
        return Lib()


class FakeClient:
    def __init__(self):
        self.batches = []
        self.next_id = 100

    def apply_batch_sync(self, cmds, do_tick):
        self.batches.append(cmds)

        class R:
            def __init__(self, i):
                self.error, self.actor_id = "", i
        out = []
        for _ in cmds:
            out.append(R(self.next_id))
            self.next_id += 1
        return out


class FakeEgo:
    def __init__(self):
        self.x = 0.0

    def get_transform(self):
        return carla.Transform(carla.Location(self.x, 0.0, 0.0))


def plan(a, width=3.5):
    """A straight road along CARLA's +x from x = 50 (yaw 0): its right is +y."""
    (n,) = scenario.check_actors([a])
    length = 1.0 if n["type"] == "pedestrian" else 400.0
    path = [(50.0 + i * scenario.PATH_STEP, 0.0 + (3.5 * n["lane"] if n["type"] in ("slow_car", "cut_in") else 0.0), 0.0, 0.0)
            for i in range(int(length / scenario.PATH_STEP) + 1)]
    return dict(n, number=1, path=path, width=width, lane_id=-2, road_id=1)


class _Cmd:
    """Stands in for carla.command's commands (they take real blueprints only)."""

    def __init__(self, *args):
        self.args = args
        if len(args) == 2 and isinstance(args[0], FakeBP):
            self.blueprint, self.transform = args
        elif len(args) == 2 and isinstance(args[0], int):
            self.actor_id, self.transform = args

    def then(self, other):
        self.next = other
        return self


FAKE_COMMAND = types.SimpleNamespace(SpawnActor=_Cmd, SetSimulatePhysics=_Cmd, ApplyTransform=_Cmd, FutureActor=0)


class MotionTests(unittest.TestCase):
    def setUp(self):
        p = mock.patch.object(scenario.carla, "command", FAKE_COMMAND)
        p.start()
        self.addCleanup(p.stop)

    def movers(self, a):
        self.client, self.ego = FakeClient(), FakeEgo()
        m = scenario.Movers(self.client, FakeWorld(), [plan(a)], self.ego)
        return m

    def last_loc(self):
        return self.client.batches[-1][0].transform.location

    def test_spawned_like_the_props(self):
        m = self.movers(actor())
        (cmd,) = self.client.batches[0]
        self.assertEqual(m.ids, [100])
        self.assertEqual(cmd.blueprint.model, scenario.CAR_MODEL)
        self.assertEqual(cmd.blueprint.attrs["role_name"], scenario.ROLE)

    def test_slow_car(self):
        m = self.movers(actor(speed_kmh=36))
        m.step(0.05)  # places it
        x0 = self.last_loc().x
        for _ in range(20):
            m.step(0.05)
        self.assertAlmostEqual(self.last_loc().x - x0, 10.0, places=4)  # 10 m/s for 1 s
        self.assertEqual(m.velocities[100], (10.0, 0.0, 0.0))

    def test_lead_brake(self):
        m = self.movers(actor(type="lead_brake", speed_kmh=36, trigger_m=20, param=5))
        m.step(0.05)
        for _ in range(20):
            m.step(0.05)
        self.assertEqual(m.velocities[100][0], 10.0)  # the ego (at 0) is 60 m away: not yet
        self.ego.x = self.last_loc().x - 19.0         # within 20 m
        for _ in range(60):
            m.step(0.05)
        self.assertEqual(m.velocities[100][0], 0.0)  # 5 m/s^2 from 10 m/s: stopped after 2 s
        x = self.last_loc().x
        m.step(0.05)
        self.assertEqual(self.last_loc().x, x)

    def test_cut_in_into_the_ego_lane(self):
        m = self.movers(actor(type="cut_in", lane=-1, speed_kmh=36, trigger_m=10, param=2.0))
        m.step(0.05)
        self.assertAlmostEqual(self.last_loc().y, -3.5)  # a lane to the left (CARLA's y is right)
        self.ego.x = self.last_loc().x - 5.0
        ys = []
        for _ in range(60):
            m.step(0.05)
            ys.append(self.last_loc().y)
        self.assertAlmostEqual(ys[-1], 0.0, places=6)  # in the ego's lane after 2 s
        self.assertTrue(all(b >= a - 1e-9 for a, b in zip(ys, ys[1:])))  # steadily to the right
        self.assertEqual(m.velocities[100][1], 0.0)  # done: no sideways speed

    def test_pedestrian(self):
        m = self.movers(actor(type="pedestrian", lane=1, speed_kmh=3.6, trigger_m=30))
        m.step(0.05)
        edge = 3.5 / 2 + scenario.WALK_MARGIN
        self.assertAlmostEqual(self.last_loc().y, edge)  # waiting on the right
        for _ in range(10):
            m.step(0.05)
        self.assertAlmostEqual(self.last_loc().y, edge)
        self.ego.x = 50.0 - 25.0
        for _ in range(20):
            m.step(0.05)
        self.assertAlmostEqual(self.last_loc().y, edge - 1.0, places=6)  # 1 m/s for 1 s, towards the left
        self.assertAlmostEqual(m.velocities[100][1], -1.0)
        for _ in range(200):
            m.step(0.05)
        self.assertAlmostEqual(self.last_loc().y, -edge)  # stops at the far side
        self.assertEqual(m.velocities[100][1], 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
