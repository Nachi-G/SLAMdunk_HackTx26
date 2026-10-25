"""Stationary fixture validator. Publishes only a separate ROS preview cloud."""
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from analyze_stationary_lidar import analyze_points

OUT = Path(__file__).resolve().parent / 'ros_lidar'


def decode_xyz(msg):
    # Handle row padding explicitly instead of assuming a tightly packed cloud.
    dtype = point_cloud2.dtype_from_fields(msg.fields, point_step=msg.point_step)
    if not {'x', 'y', 'z'}.issubset(dtype.names or ()):
        raise ValueError('XYZ fields missing')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.row_step * msg.height:
        raise ValueError('Malformed cloud dimensions/data length')
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    points = np.column_stack([values[name].ravel() for name in ('x', 'y', 'z')])
    points = points[np.isfinite(points).all(axis=1)]
    ranges = np.linalg.norm(points, axis=1)
    return points[(ranges > 0.3) & (ranges < 25)]


class Validator(Node):
    def __init__(self):
        super().__init__('stationary_lidar_validator')
        self.clock_ns = None
        self.clock_received = None
        self.last_stamp = None
        self.stamps = []
        self.ages = []
        self.geometry = []
        self.rejected = {}
        self.done = False
        self.report = None
        self.create_subscription(Clock, '/sando_perception/test_clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/sando_perception/lidar/points_raw', self.cloud,
                                 qos_profile_sensor_data)
        self.preview = self.create_publisher(PointCloud2, '/sando_perception/points_preview', 5)

    def clock(self, msg):
        value = msg.clock.sec * 10**9 + msg.clock.nanosec
        if self.clock_ns is not None and value < self.clock_ns:
            self.report = {'status': 'FAIL: simulation clock reset during test'}
            self.done = True
        self.clock_ns = value
        self.clock_received = time.monotonic()

    def reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def cloud(self, msg):
        if self.done:
            return
        try:
            if msg.header.frame_id != 'sando_lidar_link':
                raise ValueError(f'Unexpected frame: {msg.header.frame_id}')
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if stamp <= 0 or (self.last_stamp is not None and stamp <= self.last_stamp):
                raise ValueError('Zero, duplicate, or decreasing acquisition timestamp')
            if self.clock_ns is None or time.monotonic() - self.clock_received > 2.0:
                raise ValueError('No recent test simulation clock')
            age = (self.clock_ns - stamp) / 1e9
            # Independent ROS topics may arrive in either order.
            if not -0.5 <= age <= 0.5:
                raise ValueError('Cloud stamp differs from test clock by over 0.5 seconds')
            points = decode_xyz(msg)
            geometry = analyze_points(points)
            if not geometry['status'].startswith('PASS'):
                raise ValueError('Cloud does not match stationary forest geometry')
            self.last_stamp = stamp
            self.stamps.append(stamp)
            self.ages.append(age)
            self.geometry.append(geometry)
            # Explicit fixture transform. No claim of PX4 map-frame alignment.
            world_points = points + np.asarray(geometry['fixture_sensor_position_enu_m'])
            world_points = world_points[world_points[:, 2] > 0.15]
            header = Header(stamp=msg.header.stamp, frame_id='forest_test_enu')
            self.preview.publish(point_cloud2.create_cloud_xyz32(header, world_points.astype(np.float32)))
            if len(self.stamps) >= 10:
                publishers = self.get_publishers_info_by_topic('/sando_perception/lidar/points_raw')
                period = np.diff(self.stamps) / 1e9
                self.report = {
                    'status': 'PASS: stationary Gazebo to ROS point-cloud delivery',
                    'accepted_clouds': len(self.stamps), 'rejected_clouds': self.rejected,
                    'source_frame': msg.header.frame_id, 'preview_frame': header.frame_id,
                    'preview_topic': '/sando_perception/points_preview',
                    'preview_obstacle_points': int(len(world_points)),
                    'median_simulation_period_s': float(np.median(period)),
                    'maximum_simulation_period_s': float(max(period)),
                    'clock_minus_cloud_stamp_range_s': [min(self.ages), max(self.ages)],
                    'minimum_geometry_match_fraction': min(g['geometry_match_fraction'] for g in self.geometry),
                    'last_cloud_geometry': geometry,
                    'publisher_qos': [{'node': p.node_name,
                                      'reliability': p.qos_profile.reliability.name,
                                      'durability': p.qos_profile.durability.name} for p in publishers],
                    'limits': 'Static fixture only. No SANDO input or PX4 commands published. Moving sensor synchronization, PX4 local origin, persistence and unknown-space handling still require validation.',
                }
                np.savez_compressed(OUT / 'last_preview_points.npz', points=world_points, stamp_ns=stamp)
                self.done = True
        except (ValueError, KeyError, TypeError) as error:
            self.reject(str(error))


def main():
    OUT.mkdir(exist_ok=True)
    rclpy.init()
    node = Validator()
    deadline = time.monotonic() + 50
    try:
        while not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report or {'status': 'FAIL: timed out waiting for ten validated clouds',
                                 'accepted_clouds': len(node.stamps), 'rejected_clouds': node.rejected}
        (OUT / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report['status'].startswith('PASS') else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
