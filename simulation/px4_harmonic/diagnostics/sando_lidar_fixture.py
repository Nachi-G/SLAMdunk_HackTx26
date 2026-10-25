"""Stationary real-LiDAR planning fixture, restricted to isolated ROS domain 88."""
import json
import os
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
from geometry_msgs.msg import PoseStamped
from dynus_interfaces.msg import State, DynTraj
from validate_ros_lidar import decode_xyz
from analyze_stationary_lidar import analyze_points
from validate_planner_trajectory import validate

OUT = Path(os.environ.get('SANDO_TEST_OUTPUT', str(Path(__file__).resolve().parent/'sando_lidar')))


class Fixture(Node):
    def __init__(self):
        super().__init__('stationary_sando_lidar_fixture')
        self.clock_ns = None
        self.clock_received = None
        self.last_stamp = 0
        self.last_cloud_wall = None
        self.clouds = 0
        self.goal_sent = False
        self.done = False
        self.report = None
        self.rejected = {}
        self.trajectory_checks = []
        self.last_geometry = None
        self.state_pub = self.create_publisher(State,'/NX01/state',10)
        self.map_pub = self.create_publisher(PointCloud2,'/NX01/sensor_point_cloud',10)
        self.goal_pub = self.create_publisher(PoseStamped,'/NX01/term_goal',10)
        self.create_subscription(Clock,'/sando_perception/test_clock',self.clock,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/sando_perception/lidar/points_raw',self.cloud,qos_profile_sensor_data)
        self.create_subscription(DynTraj,'/trajs',self.trajectory,10)
        self.create_timer(0.05,self.state)

    def clock(self,msg):
        value=msg.clock.sec*10**9+msg.clock.nanosec
        if self.clock_ns is not None and value<self.clock_ns:
            self.report={'status':'FAIL: test simulation clock reset'}
            self.done=True
        self.clock_ns=value
        self.clock_received=time.monotonic()

    def state(self):
        msg=State()
        msg.header=Header(stamp=self.get_clock().now().to_msg(),frame_id='map')
        msg.pos.z=2.0
        msg.quat.w=1.0
        self.state_pub.publish(msg)
        if (not self.goal_sent and self.clouds>=5 and self.goal_pub.get_subscription_count()>0
                and self.map_pub.get_subscription_count()>0 and self.state_pub.get_subscription_count()>0):
            goal=PoseStamped()
            goal.header=msg.header
            goal.pose.position.x=10.0
            goal.pose.position.y=-4.44
            goal.pose.position.z=2.0
            goal.pose.orientation.w=1.0
            self.goal_pub.publish(goal)
            self.goal_sent=True

    def cloud(self,msg):
        try:
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if msg.header.frame_id!='sando_lidar_link' or stamp<=self.last_stamp:
                raise ValueError('Unexpected sensor frame or nonincreasing timestamp')
            if self.clock_ns is None or time.monotonic()-self.clock_received>2:
                raise ValueError('Missing recent simulation clock')
            if abs(self.clock_ns-stamp)>500_000_000:
                raise ValueError('Cloud and simulation clock disagree')
            points=decode_xyz(msg)
            geometry=analyze_points(points)
            if not geometry['status'].startswith('PASS'):
                raise ValueError('Sensor geometry validation failed')
            points+=np.asarray(geometry['fixture_sensor_position_enu_m'])
            points=points[points[:,2]>0.15]
            # Domain 88's map is explicitly this stationary world's ENU origin.
            # Preserve acquisition stamp; SANDO fake_sim ignores cloud timestamps.
            header=Header(stamp=msg.header.stamp,frame_id='map')
            self.map_pub.publish(point_cloud2.create_cloud_xyz32(header,points.astype(np.float32)))
            self.last_stamp=stamp
            self.last_cloud_wall=time.monotonic()
            self.last_geometry=geometry
            self.clouds+=1
        except (ValueError,KeyError,TypeError) as error:
            self.rejected[str(error)]=self.rejected.get(str(error),0)+1

    def trajectory(self,msg):
        if not self.goal_sent or self.last_cloud_wall is None or time.monotonic()-self.last_cloud_wall>2:
            return
        try:
            report=validate(msg)
        except ValueError as error:
            self.rejected[str(error)]=self.rejected.get(str(error),0)+1
            return
        self.trajectory_checks.append(report)
        if report['status'].startswith('PASS') or len(self.trajectory_checks)>=5:
            self.report={'status':'PASS: real stationary LiDAR to SANDO planning' if report['status'].startswith('PASS') else 'FAIL: planner trajectory checks',
                         'validated_sensor_clouds':self.clouds,'goal_enu_m':[10.,-4.44,2.],
                         'map_subscription_count':self.map_pub.get_subscription_count(),
                         'last_sensor_geometry':self.last_geometry,'trajectory_checks':self.trajectory_checks,
                         'rejections':self.rejected,
                         'limits':'Isolated stationary planning test. State remains fixed; no drone tracking, moving sensor transform, or closed-loop flight demonstrated.'}
            self.done=True


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='88' or not os.environ.get('GZ_PARTITION','').startswith('sando_ros_sensor_'):
        raise RuntimeError('This fixture may only run in its isolated test environment')
    OUT.mkdir(exist_ok=True)
    rclpy.init()
    node=Fixture()
    try:
        deadline=time.monotonic()+50
        while not node.done and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.2)
        report=node.report or {'status':'FAIL: isolated planning timed out','validated_sensor_clouds':node.clouds,
                              'goal_sent':node.goal_sent,'trajectory_checks':node.trajectory_checks,'rejections':node.rejected}
        (OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        return 0 if report['status'].startswith('PASS') else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    raise SystemExit(main())
