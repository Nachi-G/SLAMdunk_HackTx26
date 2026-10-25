"""Read-only seven-second probe using the same Python environment as manual mode."""
import collections
import json
import os
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry,VehicleStatus,VehicleLandDetected
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

rclpy.init();node=Node('demo_readonly_probe');gz=GzNode();counts=collections.Counter();last={};lock=threading.Lock()
def pose(message):
    with lock:
        counts['gazebo_pose']+=1;last['gazebo_model_names']=[p.name for p in message.pose if 'x500' in p.name or 'demo_solid' in p.name]
        last['gazebo_time_s']=message.header.stamp.sec+message.header.stamp.nsec/1e9

def receive(kind,message):
    counts[kind]+=1
    if kind=='status':last.update(armed=message.arming_state,nav_state=message.nav_state,failsafe=message.failsafe)
    if kind=='landed':last['landed']=message.landed
    if kind=='odometry':last['odometry_timestamp']=message.timestamp
subscriptions=[node.create_subscription(cls,topic,lambda m,k=kind:receive(k,m),qos_profile_sensor_data) for kind,cls,topic in (
    ('odometry',VehicleOdometry,'/fmu/out/vehicle_odometry'),('status',VehicleStatus,'/fmu/out/vehicle_status_v4'),('landed',VehicleLandDetected,'/fmu/out/vehicle_land_detected'))]
gz.subscribe(Pose_V,'/world/default/pose/info',pose)
until=time.monotonic()+7
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
print(json.dumps({'received_counts':dict(counts),'last':last,'domain':os.environ.get('ROS_DOMAIN_ID','0'),'partition':os.environ.get('GZ_PARTITION','default')},indent=2))
gz.unsubscribe('/world/default/pose/info');node.destroy_node();rclpy.shutdown()
