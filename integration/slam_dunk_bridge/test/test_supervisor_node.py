"""ROS adapter wiring with strict generated-message-like float setters.

These are stub checks, not ROS/Gazebo integration tests.
"""
import json
import math
import struct
import sys
import time
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from slam_dunk_bridge import supervisor_node


class FloatPoint:
    def __setattr__(self, name, value):
        if name in ('x','y','z') and not isinstance(value,float):
            raise AssertionError('ROS float64 setter requires float')
        object.__setattr__(self,name,value)


class PoseStamped:
    def __init__(self):
        self.header=NS(frame_id=None,stamp=None)
        self.pose=NS(position=FloatPoint(),orientation=NS(w=0.0))


class FakeServer:
    def __init__(self,*args,**kwargs): self.port=8766
    def start(self): pass
    def stop(self): pass


class SupervisorNodeTests(unittest.TestCase):
    def test_ros_adapter_float_goals_frames_and_admission_300(self):
        for i in range(300):
            with self.subTest(i=i):
                calls=[]
                class Node:
                    def __init__(self,name):
                        self.name=name;self.params={};self.messages=[];self.warnings=[]
                        self.publishers=[NS(node_name=name,node_namespace='/')]
                    def declare_parameter(self,name,value):
                        self.params[name]={'enable_exploration':True,'waypoints_json':json.dumps([[8,0,2],[8,8,2]])}.get(name,value)
                    def get_parameter(self,name): return NS(value=self.params[name])
                    def create_publisher(self,*args): return NS(publish=lambda message:self.messages.append(message))
                    def create_subscription(self,*args): return NS()
                    def get_clock(self): return NS(now=lambda:NS(nanoseconds=100000000000,to_msg=lambda:NS(sec=100,nanosec=0)))
                    def get_logger(self): return NS(info=lambda *a:None,warning=lambda *a:self.warnings.append(a))
                    def get_publishers_info_by_topic(self,topic): return self.publishers
                    def get_name(self): return self.name
                    def get_namespace(self): return '/'
                    def destroy_node(self): calls.append('destroy')
                rclpy=types.ModuleType('rclpy');rclpy.init=lambda **kw:calls.append('init');rclpy.shutdown=lambda:calls.append('shutdown')
                def spin(node):
                    header=NS(frame_id='map',stamp=NS(sec=100,nanosec=0))
                    pose=NS(header=header,pose=NS(pose=NS(position=NS(x=i/100,y=0,z=2),orientation=NS(x=0,y=0,z=0,w=1))))
                    fields=[NS(name=name,datatype=7,count=1,offset=j*4) for j,name in enumerate(('x','y','z'))]
                    cloud=NS(header=header,width=1,height=1,point_step=12,row_step=12,fields=fields,is_bigendian=False,data=struct.pack('<fff',1,2,3))
                    node.receive_pose(pose);node.receive_map(cloud)
                    self.assertFalse(node.supervisor.status()['ready'])
                    node.check_authority();node.authority_quiet_since=time.monotonic()-1.1;node.last_authority_check=-math.inf;node.check_authority()
                    self.assertTrue(node.supervisor.status()['ready']);node.supervisor.tick()
                    self.assertEqual(node.messages[-1].pose.position.x,8.0)
                    self.assertEqual(node.messages[-1].header.frame_id,'map')
                    self.assertIsInstance(node.messages[-1].pose.position.y,float)
                    node.publishers.append(NS(node_name=node.name,node_namespace='/'))
                    node.last_authority_check=-math.inf;node.check_authority()
                    self.assertFalse(node.supervisor.status()['ready'])
                    node.publishers.pop();node.authority_quiet_since=time.monotonic()-1.1
                    node.last_authority_check=-math.inf;node.check_authority()
                    self.assertTrue(node.supervisor.status()['ready'])
                    previous=dict(node.supervisor.received)
                    header.frame_id='other';header.stamp.nanosec=100000000
                    node.receive_pose(pose);node.receive_map(cloud)
                    self.assertEqual(node.supervisor.received,previous)
                    header.frame_id='map';cloud.data=b'broken';node.receive_map(cloud)
                    self.assertEqual(node.supervisor.received,previous)
                    node.publishers.append(NS(node_name='goal_sender',node_namespace='/'))
                    node.last_authority_check=-math.inf;node.check_authority();node.supervisor.tick()
                    self.assertFalse(node.supervisor.status()['ready']);self.assertTrue(node.warnings)
                rclpy.spin=spin
                node_mod=types.ModuleType('rclpy.node');node_mod.Node=Node
                qos_mod=types.ModuleType('rclpy.qos');qos_mod.QoSProfile=lambda **kw:NS(**kw);qos_mod.ReliabilityPolicy=NS(BEST_EFFORT=1);qos_mod.HistoryPolicy=NS(KEEP_LAST=1)
                geometry=types.ModuleType('geometry_msgs.msg');geometry.PoseStamped=PoseStamped
                nav=types.ModuleType('nav_msgs.msg');nav.Odometry=NS
                sensor=types.ModuleType('sensor_msgs.msg');sensor.PointCloud2=NS
                modules={'rclpy':rclpy,'rclpy.node':node_mod,'rclpy.qos':qos_mod,'geometry_msgs.msg':geometry,'nav_msgs.msg':nav,'sensor_msgs.msg':sensor}
                with patch.dict(sys.modules,modules),patch.object(supervisor_node,'CommandServer',FakeServer):
                    supervisor_node.main()
                self.assertEqual(calls,['init','destroy','shutdown'])


if __name__=='__main__': unittest.main()
