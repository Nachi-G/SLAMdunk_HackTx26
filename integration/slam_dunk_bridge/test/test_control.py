import math
import unittest
from types import SimpleNamespace as NS

from slam_dunk_bridge.control import SteeringSupervisor
from slam_dunk_bridge.supervisor_node import validate_source_header


class Clock:
    def __init__(self): self.now = 10.
    def __call__(self): return self.now
    def advance(self, seconds): self.now += seconds


def fixture(yaw=0., position=(0, 0, 2), **kwargs):
    clock, goals = Clock(), []
    sup = SteeringSupervisor(lambda p, f: goals.append((p, f)), clock=clock,
                             waypoints=((8, 0, 2), (8, 8, 2)), enabled=True, **kwargs)
    sup.update_pose(position, (0, 0, math.sin(yaw/2), math.cos(yaw/2)), 100, "map")
    sup.update_map(100, "map")
    sup.tick()
    return sup, clock, goals


def start(sup, direction="forward", request="press", owner="one"):
    return sup.handle(owner, {"protocol_version": 1, "type": "steer_start", "request_id": request,
                              "direction": direction, "epoch": sup.epoch})


def lease_message(status, kind, seq=1):
    return {"protocol_version": 1, "type": kind, "lease_id": status["lease_id"],
            "epoch": status["epoch"], "seq": seq}


class ControlTests(unittest.TestCase):
    def test_source_age_consumes_freshness_budget_300(self):
        for i in range(300):
            with self.subTest(i=i):
                supervisor, clock, _ = fixture()
                age = 1.1 + (i % 80)/100.
                self.assertTrue(supervisor.update_pose((0,0,2),(0,0,0,1),101,"map",source_age_seconds=age))
                self.assertTrue(supervisor.update_map(101,"map",source_age_seconds=age))
                self.assertTrue(supervisor.status()["ready"])
                clock.advance(2.-age+.001)
                supervisor.tick()
                self.assertFalse(supervisor.status()["ready"])
                self.assertEqual(supervisor.status()["mode"],"waiting")
                received=dict(supervisor.received)
                self.assertFalse(supervisor.update_map(102,"map",source_age_seconds=float('nan')))
                self.assertFalse(supervisor.update_pose((0,0,2),(0,0,0,1),102,"map",source_age_seconds=3.))
                self.assertEqual(supervisor.received,received)

    def test_body_directions_and_yaw_are_fixed_for_press_300(self):
        for i in range(300):
            with self.subTest(i=i):
                yaw = (i-150)*math.pi/73
                s, clock, goals = fixture(yaw=yaw)
                direction = tuple(s.DIRECTIONS)[i % 4]
                result = start(s, direction, f"press-{i}")
                self.assertTrue(result["accepted"])
                dx, dy = s.DIRECTIONS[direction]
                expected = (3*(math.cos(yaw)*dx-math.sin(yaw)*dy),
                            3*(math.sin(yaw)*dx+math.cos(yaw)*dy), 2)
                for a, b in zip(goals[-1][0], expected): self.assertAlmostEqual(a, b)
                clock.advance(.21)
                s.update_pose((.4, .1, 2), (0, 0, 1, 0), 101, "map")
                s.tick()
                for a, b in zip(goals[-1][0], (expected[0]+.4, expected[1]+.1, 2)):
                    self.assertAlmostEqual(a, b)

    def test_release_expiry_disconnect_publish_autonomy_300(self):
        for i in range(300):
            with self.subTest(i=i):
                s, clock, goals = fixture(yaw=i/31)
                status = start(s, request=f"p{i}")
                before = len(goals)
                if i % 3 == 0:
                    result = s.handle("one", lease_message(status, "steer_release"))
                    self.assertTrue(result["accepted"])
                elif i % 3 == 1:
                    clock.advance(.501)
                    s.tick()
                else:
                    s.disconnect("one")
                self.assertEqual(s.status()["mode"], "autonomous")
                self.assertIsNone(s.status()["lease_id"])
                self.assertEqual(s.epoch, status["epoch"]+1)
                self.assertEqual(len(goals), before+1)
                self.assertEqual(goals[-1][0], (8, 0, 2))
                self.assertFalse(s.handle("one", lease_message(status, "steer_renew", 99))["accepted"])

    def test_renew_sequence_owner_and_pause_300(self):
        for i in range(300):
            with self.subTest(i=i):
                s, clock, _ = fixture()
                status = start(s, request=f"p{i}")
                clock.advance(.3)
                self.assertFalse(s.handle("two", lease_message(status, "steer_renew", 1))["accepted"])
                self.assertTrue(s.handle("one", lease_message(status, "steer_renew", 1))["accepted"])
                self.assertFalse(s.handle("one", lease_message(status, "steer_renew", 1))["accepted"])
                self.assertFalse(start(s, "left", f"other{i}", "two")["accepted"])
                clock.advance(.3)
                s.tick()
                self.assertEqual(s.status()["mode"], "steering")
                self.assertFalse(s.handle("one", {"protocol_version":1,"type":"pause", "epoch":s.epoch,"request_id":f"pause{i}"})["accepted"])
                self.assertFalse(s.status()["capabilities"]["pause"])
                clock.advance(.21)
                s.tick()
                self.assertEqual(s.status()["mode"], "autonomous")

    def test_cancel_before_and_after_start_race_300(self):
        for i in range(300):
            with self.subTest(i=i):
                s, _, goals = fixture()
                initial = s.epoch
                cancel = {"protocol_version":1,"type":"steer_cancel","epoch":initial,"request_id":f"p{i}"}
                if i % 2:
                    status = start(s, request=f"p{i}")
                    self.assertIsNotNone(status["lease_id"])
                self.assertTrue(s.handle("one", cancel)["accepted"])
                self.assertEqual(s.epoch, initial+1)
                self.assertFalse(s.handle("one", {"protocol_version":1,"type":"steer_start","epoch":initial,"request_id":f"p{i}","direction":"forward"})["accepted"])
                self.assertIsNone(s.lease)
                self.assertEqual(goals[-1][0], (8, 0, 2))

    def test_stale_and_rejected_inputs_do_not_refresh_success_300(self):
        invalid = (math.nan, math.inf, -1, 0, 100, True, 10**400)
        for i in range(300):
            with self.subTest(i=i):
                s, clock, goals = fixture()
                self.assertTrue(start(s, request=f"p{i}")["accepted"])
                clock.advance(2.1)
                self.assertFalse(s.update_map(invalid[i%len(invalid)], "map"))
                self.assertFalse(s.update_pose((0, 0, math.nan), (0,0,0,1), 101, "map"))
                self.assertFalse(s.update_pose((0,0,2), (0,0,0,0), 101, "map"))
                self.assertFalse(s.update_pose((0,0,2), (0,0,0,1), 101, "wrong"))
                count = len(goals)
                s.tick()
                self.assertEqual(s.status()["mode"], "waiting")
                self.assertFalse(s.status()["ready"])
                self.assertEqual(len(goals), count)
                self.assertIn("prior SANDO goal", s.status()["reason"])
                self.assertFalse(start(s, request=f"new{i}")["accepted"])

    def test_configuration_bounds_rate_and_fault_300(self):
        for i in range(300):
            with self.subTest(i=i):
                with self.assertRaises(ValueError): SteeringSupervisor(lambda *_:None, lookahead=1+i%2)
                with self.assertRaises(ValueError): SteeringSupervisor(lambda *_:None, waypoints=((math.nan,0,1),))
                with self.assertRaises(ValueError): SteeringSupervisor(lambda *_:None, bounds=(0,1,0,1,0,10**400))
                s, clock, goals = fixture(position=(198,0,2))
                self.assertFalse(start(s, request=f"p{i}")["accepted"])
                s, clock, goals = fixture()
                status=start(s, request=f"rate{i}")
                count=len(goals)
                for step in range(3):
                    clock.advance(.04)
                    s.update_pose((.2*(step+1),0,2), (0,0,0,1), 101+step, "map")
                    s.tick()
                self.assertEqual(len(goals), count)
                clock.advance(.1);s.tick();self.assertEqual(len(goals), count+1)
                s.publish_goal=lambda *_: (_ for _ in ()).throw(RuntimeError("sink down"))
                s.handle("one", lease_message(status,"steer_release"))
                self.assertEqual(s.status()["mode"], "fault")
                self.assertFalse(s.status()["ready"])

    def test_source_age_and_unconfigured_startup_300(self):
        for i in range(300):
            with self.subTest(i=i):
                now=100000000000+i*1000000000
                header=NS(stamp=NS(sec=now//1000000000,nanosec=now%1000000000))
                self.assertEqual(validate_source_header(header, now), now/1e9)
                with self.assertRaises(ValueError): validate_source_header(header, now+3000000000)
                with self.assertRaises(ValueError): validate_source_header(header, now-300000000)
                goals=[];s=SteeringSupervisor(lambda *a:goals.append(a))
                s.tick();self.assertFalse(s.status()["ready"]);self.assertEqual(goals,[])
                self.assertFalse(start(s, request=f"startup{i}")["accepted"])

    def test_malformed_protocol_300(self):
        bad = (None, [], "bad", {}, {"protocol_version":True,"type":"steer_start"},
               {"protocol_version":1,"type":"steer_start","epoch":True,"direction":"forward","request_id":"p"},
               {"protocol_version":1,"type":"steer_start","epoch":0,"direction":{},"request_id":"p"},
               {"protocol_version":1,"type":"steer_start","epoch":0,"direction":"forward","request_id":"a"*200})
        for i in range(300):
            with self.subTest(i=i):
                s, _, _=fixture()
                response=s.handle("one", bad[i%len(bad)])
                self.assertFalse(response["accepted"])
                self.assertIsNone(response["lease_id"])


if __name__ == "__main__": unittest.main()
