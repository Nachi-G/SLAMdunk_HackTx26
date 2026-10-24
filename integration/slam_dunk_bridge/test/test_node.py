"""Adapter contract tests using ROS stubs; no claim of ROS integration."""

import math
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

from slam_dunk_bridge import node as adapter
from test_core import cloud


def module(name, **values):
    result = ModuleType(name)
    result.__dict__.update(values)
    return result


class AdapterTests(unittest.TestCase):
    def run_node(self, exercise):
        calls = []
        class FakeNode:
            def __init__(self, name): self.parameters = {}; self.subscriptions = []
            def declare_parameter(self, key, value): self.parameters[key] = value
            def get_parameter(self, key): return NS(value=self.parameters[key])
            def create_subscription(self, message_type, topic, callback, qos):
                self.subscriptions.append((topic, qos)); return NS()
            def create_timer(self, period, callback): return NS()
            def get_logger(self): return NS(info=lambda s: None, warning=lambda s: None)
            def destroy_node(self): pass
        class FakeBuffer:
            def __init__(self, **kwargs): pass
            def lookup_transform(self, target, source, stamp, **kwargs):
                calls.append((target, source, stamp))
                if source == "missing": raise LookupError("Unavailable TF")
                return NS(transform=NS(translation=NS(x=1,y=2,z=3),
                                       rotation=NS(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4))))
        class FakeServer:
            error = None
            def __init__(self, *args): pass
            def start(self): pass
            def stop(self): pass
        modules = {
            "rclpy": module("rclpy", init=lambda **kw: None,
                             spin=lambda node: exercise(node,calls), ok=lambda: True,
                             shutdown=lambda: None),
            "rclpy.node": module("rclpy.node", Node=FakeNode),
            "rclpy.qos": module("rclpy.qos", QoSProfile=lambda **kw:NS(**kw),
                                 ReliabilityPolicy=NS(BEST_EFFORT="best_effort"),
                                 HistoryPolicy=NS(KEEP_LAST="keep_last")),
            "rclpy.time": module("rclpy.time", Time=NS(from_msg=lambda stamp: stamp)),
            "rclpy.duration": module("rclpy.duration", Duration=lambda **kw: NS(**kw)),
            "nav_msgs": module("nav_msgs"), "nav_msgs.msg": module("nav_msgs.msg",Odometry=object),
            "sensor_msgs": module("sensor_msgs"), "sensor_msgs.msg": module("sensor_msgs.msg",PointCloud2=object),
            "tf2_ros": module("tf2_ros",Buffer=FakeBuffer,TransformListener=lambda *args: None),
        }
        with patch.dict(sys.modules, modules), patch.object(adapter,"BridgeServer",FakeServer):
            adapter.main()

    @staticmethod
    def map_message(stamp=1, frame="map", points=None):
        message=cloud(points if points is not None else [(1,2,3)])
        message.header=NS(stamp=NS(sec=stamp,nanosec=0),frame_id=frame)
        return message

    def test_snapshot_supersede_clear_and_qos(self):
        def exercise(node,calls):
            self.assertTrue(all(qos.depth==1 and qos.reliability=="best_effort"
                                for _,qos in node.subscriptions))
            node.receive("map",self.map_message())
            node.receive("map",self.map_message(2,points=[]))
            node.process()
            self.assertEqual(node.counters["map_superseded"],1)
            self.assertIsNotNone(node.accepted["map"])
            self.assertIn('"points":[]',node.store.snapshot()["map"][1])
            self.assertEqual(calls,[])
        self.run_node(exercise)

    def test_rejection_preserves_success_age_and_timestamp(self):
        def exercise(node,calls):
            node.receive("map",self.map_message(5)); node.process()
            accepted=node.accepted["map"]
            for message in (self.map_message(4),self.map_message(0,"sensor"),
                            self.map_message(6,"missing")):
                node.receive("map",message); node.process()
            self.assertEqual(node.counters["map_rejected"],3)
            self.assertEqual(node.accepted["map"],accepted)
            self.assertEqual(node.maps.revision,1)
            self.assertEqual(len(calls),1)
            self.assertEqual(calls[0][2].sec,6)
        self.run_node(exercise)

    def test_pose_uses_timestamped_transform(self):
        def exercise(node,calls):
            message=NS(header=NS(stamp=NS(sec=7,nanosec=123),frame_id="sensor"),
                       child_frame_id="base_link",pose=NS(pose=NS(position=NS(x=1,y=0,z=0),
                                                        orientation=NS(x=0,y=0,z=0,w=1))))
            node.receive("pose",message);node.process()
            import json
            payload=json.loads(node.store.snapshot()["pose"][1])
            for actual,expected in zip(payload["position"],(1,3,3)):
                self.assertAlmostEqual(actual,expected)
            self.assertEqual(calls[0][2].nanosec,123)
            self.assertEqual(payload["frame_id"],"map")
        self.run_node(exercise)


if __name__ == "__main__": unittest.main()
