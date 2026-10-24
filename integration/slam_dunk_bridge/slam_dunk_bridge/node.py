"""Read-only ROS 2 adapter. ROS imports are isolated from demo/core/tests."""

import math
import time

from .core import (LatestStore, MapPublisher, decode_xyz, health_message,
                   quaternion_product, stamp_dict, transform_point)
from .transport import BridgeServer


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import Buffer, TransformListener

    class LiveBridge(Node):
        def __init__(self):
            super().__init__("slam_dunk_bridge")
            defaults = {"map_topic": "/NX01/occupancy_grid",
                        "pose_topic": "/NX01/visual_slam/odom", "map_frame": "map",
                        "host": "127.0.0.1", "port": 8765, "rate_hz": 10.0,
                        "max_points": 20000, "max_input_points": 2000000,
                        "max_input_bytes": 64000000, "max_clients": 8}
            for key, value in defaults.items():
                self.declare_parameter(key, value)
            self.config = {key: self.get_parameter(key).value for key in defaults}
            c = self.config
            if (not c["map_frame"] or c["map_frame"].startswith("/") or
                    min(c["max_points"], c["max_input_points"], c["max_input_bytes"]) < 1):
                raise ValueError("Invalid map frame or point/byte limits")
            self.store = LatestStore()
            self.maps = MapPublisher(self.store, c["map_frame"], c["max_points"])
            self.latest = {"map": None, "pose": None}
            self.accepted = {"map": None, "pose": None}
            self.stamps = {"map": -1, "pose": -1}
            self.counters = {"map_received": 0, "pose_received": 0,
                             "map_superseded": 0, "pose_superseded": 0,
                             "map_rejected": 0, "pose_rejected": 0}
            self.tf = Buffer(cache_time=Duration(seconds=10))
            self.listener = TransformListener(self.tf, self)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.map_sub = self.create_subscription(PointCloud2, c["map_topic"],
                                                    lambda m: self.receive("map", m), qos)
            self.pose_sub = self.create_subscription(Odometry, c["pose_topic"],
                                                     lambda m: self.receive("pose", m), qos)
            self.server = BridgeServer(self.store, c["host"], c["port"], c["rate_hz"], c["max_clients"])
            self.server.start()
            self.timer = self.create_timer(1/c["rate_hz"], self.process)
            self.health_timer = self.create_timer(1.0, self.health)
            self.last_warning = 0.0
            self.get_logger().info(f"Read-only map viewer: http://{c['host']}:{c['port']}/ ; WebSocket /stream")

        def receive(self, kind, message):
            self.counters[kind+"_received"] += 1
            if self.latest[kind] is not None:
                self.counters[kind+"_superseded"] += 1
            self.latest[kind] = message

        def get_transform(self, message, kind):
            header = message.header
            stamp = int(header.stamp.sec)*1000000000 + int(header.stamp.nanosec)
            if not header.frame_id or header.frame_id.startswith("/") or stamp <= 0:
                raise ValueError("Missing/invalid frame or zero capture/publication timestamp")
            if stamp <= self.stamps[kind]:
                raise ValueError("Duplicate or out-of-order timestamp")
            if header.frame_id == self.config["map_frame"]:
                return stamp, (0., 0., 0.), (0., 0., 0., 1.)
            # Never substitute latest TF. A failed timestamped lookup rejects this update.
            result = self.tf.lookup_transform(self.config["map_frame"], header.frame_id,
                                              Time.from_msg(header.stamp), timeout=Duration(seconds=0))
            t, q = result.transform.translation, result.transform.rotation
            translation, rotation = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in translation):
                raise ValueError("Non-finite transform translation")
            return stamp, translation, rotation

        def process(self):
            for kind in ("map", "pose"):
                message, self.latest[kind] = self.latest[kind], None
                if message is None:
                    continue
                try:
                    stamp, translation, rotation = self.get_transform(message, kind)
                    if kind == "map":
                        if len(message.data) > self.config["max_input_bytes"]:
                            raise ValueError("Cloud exceeds input byte limit")
                        points, statistics = decode_xyz(message, self.config["max_points"],
                                                        self.config["max_input_points"])
                        points = [transform_point(p, translation, rotation) for p in points]
                        self.maps.replace(points, stamp_dict(message.header.stamp),
                                          self.config["map_topic"], statistics)
                    else:
                        p, q = message.pose.pose.position, message.pose.pose.orientation
                        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                            raise ValueError("Non-finite pose")
                        self.store.publish({"type": "pose", "frame_id": self.config["map_frame"],
                                            "child_frame_id": message.child_frame_id,
                                            "stamp": stamp_dict(message.header.stamp),
                                            "source": self.config["pose_topic"],
                                            "position": transform_point((p.x,p.y,p.z), translation, rotation),
                                            "orientation": quaternion_product(rotation, (q.x,q.y,q.z,q.w))})
                    self.stamps[kind] = stamp
                    self.accepted[kind] = time.monotonic()
                except Exception as exc:
                    self.counters[kind+"_rejected"] += 1
                    if time.monotonic() - self.last_warning > 1:
                        self.get_logger().warning(f"Rejected {kind} update: {exc}")
                        self.last_warning = time.monotonic()

        def health(self):
            self.store.publish(health_message(self.accepted, self.counters, self.config["map_frame"]))
            if self.server.error:
                raise RuntimeError("WebSocket server stopped unexpectedly") from self.server.error

    rclpy.init(args=args)
    node = None
    try:
        node = LiveBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.server.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
