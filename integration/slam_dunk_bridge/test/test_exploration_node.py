"""Timestamped sensor/TF wiring tests with ROS stubs, not simulator tests."""

import sys
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from slam_dunk_bridge import supervisor_node
from test_core import cloud
from test_supervisor_node import PoseStamped, FakeServer


def module(name, **values):
    result = types.ModuleType(name)
    result.__dict__.update(values)
    return result


class NoThread:
    def __init__(self, **kwargs):
        pass

    def start(self):
        pass

    def join(self, **kwargs):
        pass


class ExplorationAdapterTests(unittest.TestCase):
    def run_node(self, exercise, overrides=None):
        config = {"autonomy_mode": "frontier", "enable_exploration": True,
                  "bounds": [0., 12., 0., 8., 0., 3.], "exploration_altitude_m": 1.5,
                  "exploration_clearance_m": 0., "exploration_max_rays": 16}
        config.update(overrides or {})
        lookups = []

        class Node:
            def __init__(self, name):
                self.name, self.params, self.subscriptions = name, {}, []
                self.messages, self.warnings, self.timers = [], [], []

            def declare_parameter(self, name, value):
                self.params[name] = config.get(name, value)

            def get_parameter(self, name):
                return NS(value=self.params[name])

            def create_publisher(self, *args):
                return NS(publish=self.messages.append)

            def create_subscription(self, message_type, topic, callback, qos):
                self.subscriptions.append((topic, qos))
                return NS()

            def create_timer(self, period, callback):
                self.timers.append((period, callback))
                return NS()

            def get_clock(self):
                return NS(now=lambda: NS(nanoseconds=100000000000, to_msg=lambda: NS(sec=100, nanosec=0)))

            def get_logger(self):
                return NS(info=lambda *a: None, warning=lambda *a: self.warnings.append(a))

            def get_publishers_info_by_topic(self, topic):
                return [NS(node_name=self.name, node_namespace="/")]

            def get_name(self):
                return self.name

            def get_namespace(self):
                return "/"

            def destroy_node(self):
                pass

        class Buffer:
            def lookup_transform(self, target, source, stamp, **kwargs):
                lookups.append((target, source, stamp))
                if source == "missing":
                    raise LookupError("no timestamped TF")
                translation = (0., 0., 0.) if source == "map" else (2.5, 3.5, 1.5)
                return NS(transform=NS(translation=NS(**dict(zip(("x", "y", "z"), translation))),
                    rotation=NS(x=0., y=0., z=0., w=1.)))

        modules = {
            "rclpy": module("rclpy", init=lambda **kw: None, spin=lambda node: exercise(node, lookups), shutdown=lambda: None),
            "rclpy.node": module("rclpy.node", Node=Node),
            "rclpy.qos": module("rclpy.qos", QoSProfile=lambda **kw: NS(**kw),
                HistoryPolicy=NS(KEEP_LAST=1), ReliabilityPolicy=NS(BEST_EFFORT=1)),
            "rclpy.time": module("rclpy.time", Time=NS(from_msg=lambda stamp: stamp)),
            "rclpy.duration": module("rclpy.duration", Duration=lambda **kw: NS(**kw)),
            "geometry_msgs.msg": module("geometry_msgs.msg", PoseStamped=PoseStamped),
            "nav_msgs.msg": module("nav_msgs.msg", Odometry=NS),
            "sensor_msgs.msg": module("sensor_msgs.msg", PointCloud2=NS),
            "tf2_ros": module("tf2_ros", Buffer=Buffer, TransformListener=lambda *args: NS()),
        }
        with patch.dict(sys.modules, modules), patch.object(supervisor_node, "CommandServer", FakeServer), \
                patch.object(supervisor_node.threading, "Thread", NoThread):
            supervisor_node.main()

    @staticmethod
    def scan(frame="lidar", sec=100, points=None):
        message = cloud(points if points is not None else [(2., 0., 0.), (0., 2., 0.)])
        message.header = NS(frame_id=frame, stamp=NS(sec=sec, nanosec=123))
        return message

    def test_latest_scan_timestamped_tf_and_bounded_input(self):
        def exercise(node, lookups):
            self.assertTrue(any(topic == "/NX01/mid360_PointCloud2" for topic, _ in node.subscriptions))
            self.assertTrue(all(qos.depth == 1 for _, qos in node.subscriptions))
            node.receive_scan(self.scan(points=[(1., 0., 0.)]))
            node.receive_scan(self.scan())
            node.process_scan()
            self.assertEqual(node.observation_counters["superseded"], 1)
            self.assertEqual(node.observation_counters["accepted"], 1)
            self.assertEqual(lookups[0][0:2], ("map", "lidar"))
            self.assertEqual(lookups[0][2].nanosec, 123)
            self.assertTrue(node.explorer.ready())
            self.assertGreater(node.explorer.status()["free_voxels"], 0)
            before = node.explorer.last_scan
            node.receive_scan(self.scan("missing"))
            node.process_scan()
            self.assertEqual(node.observation_counters["rejected"], 1)
            self.assertEqual(node.explorer.last_scan, before)
        self.run_node(exercise)

    def test_stale_world_frame_and_malformed_scan_do_not_refresh(self):
        def exercise(node, lookups):
            for message in (self.scan(sec=90), self.scan("map"), self.scan(points=[])):
                node.receive_scan(message)
                node.process_scan()
            self.assertEqual(node.observation_counters["rejected"], 3)
            self.assertFalse(node.explorer.ready())
            self.assertFalse(node.supervisor.status()["ready"])
            self.assertEqual(node.messages, [])
        self.run_node(exercise)

    def test_world_coordinate_cloud_uses_separate_sensor_origin(self):
        def exercise(node, lookups):
            node.receive_scan(self.scan("map", points=[(4.5, 3.5, 1.5)]))
            node.process_scan()
            self.assertEqual([item[1] for item in lookups], ["map", "lidar"])
            self.assertEqual(node.observation_counters["accepted"], 1)
            self.assertTrue(node.explorer.ready())
        self.run_node(exercise, {"exploration_sensor_origin_frame": "lidar"})


if __name__ == "__main__":
    unittest.main()
