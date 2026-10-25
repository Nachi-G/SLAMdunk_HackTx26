#!/usr/bin/env python3
"""Run on the ROS/Gazebo machine with car.launch.py already active in teleop mode.
Checks actual sensor/map messages, requested wheel motion and command expiry.
"""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan,PointCloud2,Image,Imu
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init();node=rclpy.create_node('car_runtime_smoke');seen={};odom=[];commands=[]
    for topic,kind in [('/car/scan',LaserScan),('/car/points',PointCloud2),('/car/rgb/image',Image),('/car/depth/image',Image),('/car/imu',Imu),('/map',OccupancyGrid)]:
        node.create_subscription(kind,topic,lambda msg,t=topic:seen.__setitem__(t,msg),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/car/odom',lambda msg:odom.append(msg),qos_profile_sensor_data)
    node.create_subscription(Twist,'/car/cmd_vel',lambda msg:commands.append(msg),1)
    pub=node.create_publisher(Twist,'/car/cmd_vel_teleop',1)
    def spin_until(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not predicate():rclpy.spin_once(node,timeout_sec=.05)
        assert predicate(),'runtime condition timed out'
    try:
        spin_until(lambda:len(seen)==6 and bool(odom),30.)
        assert seen['/car/scan'].header.frame_id=='laser'
        assert seen['/car/points'].header.frame_id=='lidar3d'
        assert seen['/car/scan'].ranges and seen['/car/points'].width>0
        assert any(v==0 for v in seen['/map'].data)
        start=odom[-1].pose.pose.position.x;commands.clear();end=time.monotonic()+2.
        while time.monotonic()<end:
            request=Twist();request.linear.x=.15;pub.publish(request);rclpy.spin_once(node,timeout_sec=.025)
        assert any(msg.linear.x>.02 for msg in commands),'guard did not admit motion; inspect /car/drive_status'
        assert odom[-1].pose.pose.position.x-start>.02,'wheel odometry did not advance'
        commands.clear()
        spin_until(lambda:len(commands)>5 and all(abs(m.linear.x)+abs(m.angular.z)<1e-6 for m in commands[-5:]),3.)
        print('PASS: sensors, SLAM free cells, wheel odometry, command expiry. Inspect physical chassis motion in Gazebo too.')
    finally:
        pub.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
