"""Exercise actual adapter callbacks with strict message stubs, without ROS.
This verifies field types/contracts and admission; it is not a ROS runtime test.
"""
import math
import sys
import time
import types
import unittest
from unittest.mock import patch
from slamdunk_car import sando_node

class Vector:
    def __init__(self,x=0.,y=0.,z=0.):self.x=float(x);self.y=float(y);self.z=float(z)
class Point(Vector):pass
class Quaternion:
    x=y=z=0.;w=1.
class Stamp:
    def __init__(self,seconds=10.):self.sec=int(seconds);self.nanosec=round((seconds-int(seconds))*1e9)
class Header:
    def __init__(self):self.stamp=Stamp();self.frame_id='map'
class Pose:
    def __init__(self):self.position=Point();self.orientation=Quaternion()
    def __setattr__(self,name,value):
        if name=='position' and not isinstance(value,Point):raise TypeError('Pose.position requires Point')
        super().__setattr__(name,value)
class Twist:
    def __init__(self):self.linear=Vector();self.angular=Vector()
class Odometry:
    def __init__(self):self.header=Header();self.child_frame_id='base_link';self.pose=types.SimpleNamespace(pose=Pose());self.twist=types.SimpleNamespace(twist=Twist())
class State:
    def __init__(self):self.header=Header();self.pos=Vector();self.vel=Vector();self.quat=Quaternion()
class Goal:
    def __init__(self):self.header=Header();self.p=Vector();self.v=Vector();self.a=Vector();self.j=Vector();self.yaw=self.dyaw=0.
class String:pass
class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
class Node:
    def __init__(self,name):self.pubs={}
    def create_publisher(self,kind,topic,qos):p=Publisher();self.pubs[topic]=p;return p
    def create_subscription(self,*args):pass
    def create_timer(self,*args):pass
    def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=10_000_000_000))
    def get_logger(self):return types.SimpleNamespace(warning=lambda text:None)
    def destroy_node(self):pass
class Buffer:
    def lookup_transform(self,target,source,at):
        return types.SimpleNamespace(transform=types.SimpleNamespace(translation=Vector(2,3,.6 if source=='lidar3d' else 0),rotation=Quaternion()))
class TransformException(Exception):pass


def module(name,**values):
    result=types.ModuleType(name);result.__dict__.update(values);return result


def modules(exercise):
    return {
        'rclpy':module('rclpy',init=lambda **kw:None,spin=exercise,ok=lambda:True,shutdown=lambda:None),
        'rclpy.node':module('rclpy.node',Node=Node),
        'rclpy.time':module('rclpy.time',Time=types.SimpleNamespace(from_msg=lambda s:s)),
        'rclpy.qos':module('rclpy.qos',qos_profile_sensor_data=1),
        'geometry_msgs.msg':module('geometry_msgs.msg',Twist=Twist),
        'nav_msgs.msg':module('nav_msgs.msg',OccupancyGrid=object,Odometry=Odometry),
        'sensor_msgs.msg':module('sensor_msgs.msg',PointCloud2=object),
        'sensor_msgs_py.point_cloud2':module('sensor_msgs_py.point_cloud2',read_points=lambda msg,**kw:iter(msg.points),create_cloud_xyz32=lambda h,p:types.SimpleNamespace(header=h,points=p)),
        'std_msgs.msg':module('std_msgs.msg',Header=Header,String=String),
        'dynus_interfaces.msg':module('dynus_interfaces.msg',State=State,Goal=Goal),
        'tf2_ros':module('tf2_ros',Buffer=Buffer,TransformListener=lambda *args:None,TransformException=TransformException),
    }

class AdapterContractTests(unittest.TestCase):
    def exercise_ready(self,node):
        grid=types.SimpleNamespace(header=Header(),data=[0]*400,info=types.SimpleNamespace(width=20,height=20,resolution=.2,origin=Pose()))
        grid.header.stamp=Stamp(2.);grid.info.origin.position=Point(-2,-2,0)
        node.grid(grid)
        odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=Stamp(9.99);node.odom(odom)
        cloud=types.SimpleNamespace(header=Header(),width=1,height=1,data=bytes(12),points=[(1.,0.,0.)])
        cloud.header.frame_id='lidar3d';cloud.header.stamp=Stamp(9.995);node.cloud(cloud);node.publish_map()
        return grid
    def run_node(self,callback):
        with patch.dict(sys.modules,modules(callback)):sando_node.main([])
    def test_stationary_map_is_retained_and_new_cloud_stamp_preserved(self):
        def exercise(node):
            self.exercise_ready(node)
            occupied=node.pubs['/car_sando/occupancy_grid'].messages[-1]
            unknown=node.pubs['/car_sando/unknown_grid'].messages[-1]
            self.assertEqual(occupied.header.stamp.nanosec,Stamp(9.995).nanosec)
            self.assertEqual(occupied.header.stamp.sec,9)
            self.assertEqual(occupied.header.stamp.nanosec,unknown.header.stamp.nanosec)
            self.assertTrue(node.pubs['/car_sando/state'].messages)
            pose=node.pubs['/car/map_odom'].messages[-1].pose.pose.position
            self.assertIsInstance(pose,Point);self.assertEqual((pose.x,pose.y),(2.,3.))
        self.run_node(exercise)
    def test_no_cloud_refresh_without_fresh_pose(self):
        def exercise(node):
            self.exercise_ready(node);publisher=node.pubs['/car_sando/occupancy_grid'];count=len(publisher.messages)
            node.pose_at=time.monotonic()-1.;node.publish_map();self.assertEqual(len(publisher.messages),count)
        self.run_node(exercise)
    def test_valid_planar_goal_drives_but_stale_pose_and_vertical_goals_stop(self):
        def exercise(node):
            self.exercise_ready(node)
            g=Goal();g.p=Vector(2.05,3.,.3);g.v=Vector(.1,0,0);node.goal(g)
            self.assertGreater(node.pubs['/car/cmd_vel_sando'].messages[-1].linear.x,0)
            node.pose_at=time.monotonic()-1.;g.header.stamp=Stamp(10.01);node.goal(g)
            self.assertEqual(node.pubs['/car/cmd_vel_sando'].messages[-1].linear.x,0.)
            node.pose_at=time.monotonic();g.header.stamp=Stamp(10.02);g.p.z=.8;node.goal(g)
            self.assertEqual(node.pubs['/car/cmd_vel_sando'].messages[-1].linear.x,0.)
        self.run_node(exercise)
