"""Opt-in SANDO destination supervisor. ROS/Gazebo operation is unvalidated here.

All topic inputs must already use the configured planning frame. Startup does
not command motion unless enable_exploration is true AND valid waypoints and
fresh map/pose exist. The watchdog uses monotonic wall time in its own thread,
so paused simulation time cannot preserve a browser steering lease.
"""

import json
import threading
import time

from .command_server import CommandServer, DEFAULT_ORIGINS
from .control import SteeringSupervisor
from .core import decode_xyz


def validate_source_header(header, now_ns, max_age_seconds=2.0):
    """Return source seconds; reject invalid, old, or incompatible-clock stamps."""
    sec, nano = header.stamp.sec, header.stamp.nanosec
    if (not isinstance(sec, int) or isinstance(sec, bool) or sec < 0 or
            not isinstance(nano, int) or isinstance(nano, bool) or not 0 <= nano < 1000000000):
        raise ValueError("Invalid source timestamp")
    stamp_ns = sec*1000000000+nano
    if stamp_ns <= 0 or now_ns-stamp_ns > max_age_seconds*1e9 or stamp_ns-now_ns > .2e9:
        raise ValueError("Stale/future source timestamp; align source and ROS clock")
    return stamp_ns/1e9


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    class MissionSupervisorNode(Node):
        def __init__(self):
            super().__init__("slam_dunk_mission_supervisor")
            defaults = {"host": "127.0.0.1", "port": 8766, "planning_frame": "map",
                        "pose_topic": "/NX01/visual_slam/odom", "map_topic": "/NX01/occupancy_grid",
                        "goal_topic": "/NX01/term_goal", "enable_exploration": False,
                        "waypoints_json": "[]", "lookahead_m": 3.0, "goal_seen_radius_m": 2.0,
                        "freshness_seconds": 2.0, "lease_seconds": .5,
                        "bounds": [-200., 200., -200., 200., .5, 6.],
                        "allowed_origins": list(DEFAULT_ORIGINS)}
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            c = {name: self.get_parameter(name).value for name in defaults}
            self.config = c
            self.goal_pub = self.create_publisher(PoseStamped, c["goal_topic"], 1)
            def publish_goal(target, frame):
                message = PoseStamped()
                message.header.frame_id = frame
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, target)
                message.pose.orientation.w = 1.0
                self.goal_pub.publish(message)
            self.supervisor = SteeringSupervisor(publish_goal, waypoints=json.loads(c["waypoints_json"]),
                enabled=c["enable_exploration"], frame_id=c["planning_frame"], bounds=c["bounds"],
                lookahead=c["lookahead_m"], goal_seen_radius=c["goal_seen_radius_m"],
                freshness_seconds=c["freshness_seconds"], ttl_seconds=c["lease_seconds"])
            self.supervisor.set_authority_ready(False)
            self.last_authority_check = -float("inf")
            self.authority_quiet_since = None
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pose_sub = self.create_subscription(Odometry, c["pose_topic"], self.receive_pose, qos)
            self.map_sub = self.create_subscription(PointCloud2, c["map_topic"], self.receive_map, qos)
            self.server = CommandServer(self.supervisor, c["host"], c["port"], origins=c["allowed_origins"])
            self.server.start()
            self.shutdown_watchdog = threading.Event()
            self.watchdog = threading.Thread(target=self.watch, name="steering-lease-watchdog", daemon=True)
            self.watchdog.start()
            self.get_logger().info(f"Opt-in destination supervisor: ws://{c['host']}:{self.server.port}/control; pause unsupported")

        def receive_pose(self, message):
            try:
                now_ns = self.get_clock().now().nanoseconds
                stamp = validate_source_header(message.header, now_ns, self.config["freshness_seconds"])
                p, q = message.pose.pose.position, message.pose.pose.orientation
                self.supervisor.update_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), stamp, message.header.frame_id,
                                            source_age_seconds=max(0., now_ns/1e9-stamp))
            except (ValueError, TypeError, AttributeError, OverflowError):
                pass  # Rejection never refreshes successful input age.

        def receive_map(self, message):
            try:
                now_ns = self.get_clock().now().nanoseconds
                stamp = validate_source_header(message.header, now_ns, self.config["freshness_seconds"])
                if len(message.data) > 64000000:
                    return
                decode_xyz(message, max_points=64, max_input_points=2000000)
                self.supervisor.update_map(stamp, message.header.frame_id, source_age_seconds=max(0., now_ns/1e9-stamp))
            except (ValueError, TypeError, AttributeError, OverflowError):
                pass

        def watch(self):
            while not self.shutdown_watchdog.wait(.05):
                self.check_authority()
                self.supervisor.tick()

        def check_authority(self):
            now = time.monotonic()
            if now-self.last_authority_check < .5:
                return
            self.last_authority_check = now
            try:
                publishers = self.get_publishers_info_by_topic(self.config["goal_topic"])
                competing = [p for p in publishers if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
                # Names are not unique in a ROS graph. Two supervisors can use
                # the same name/namespace; require exactly one endpoint too.
                if len(publishers) != 1 or competing:
                    self.authority_quiet_since = None
                    self.supervisor.set_authority_ready(False)
                    self.get_logger().warning("Destination control disabled: disable/remap every other term_goal writer (goal_sender/goal_relay)")
                    return
                if self.authority_quiet_since is None:
                    self.authority_quiet_since = now
                self.supervisor.set_authority_ready(now-self.authority_quiet_since >= 1.)
            except Exception:
                self.authority_quiet_since = None
                self.supervisor.set_authority_ready(False)

        def close(self):
            self.server.stop()
            self.shutdown_watchdog.set()
            self.watchdog.join(timeout=2)

    rclpy.init(args=args)
    node = None
    try:
        node = MissionSupervisorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
