"""Capture mounted LiDAR, Gazebo poses, PX4 odometry and clock sync. No publishers."""
import json
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from px4_msgs.msg import VehicleOdometry,TimesyncStatus,VehicleStatus,VehicleLandDetected
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from diagnostics.validate_ros_lidar import decode_xyz

OUT=Path(__file__).resolve().parent/'sensor_timing'


def stamp(s):return s.sec+s.nanosec/1e9


def json_value(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError(f'Unsupported capture value: {type(value).__name__}')


def save_json(path,data):
    # Serialize before opening the destination, then atomically replace it.
    text=json.dumps(data,indent=2,default=json_value)+'\n'
    temporary=path.with_suffix('.pending.json')
    temporary.write_text(text)
    temporary.replace(path)


class Capture(Node):
    def __init__(self):
        super().__init__('x500_read_only_sensor_timing_capture')
        self.data={k:[] for k in ('clouds','poses','odometry','timesync','clock','status','landed')}
        self.frames=None
        self.last_cloud=-1
        self.cloud_count=0
        self.errors=[]
        self.create_subscription(PointCloud2,'/sando_perception/lidar/points_raw',self.cloud,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/sando_perception/gazebo/poses',self.poses,qos_profile_sensor_data)
        self.create_subscription(Clock,'/sando_perception/gazebo/clock',self.clock,qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus,'/fmu/out/timesync_status',self.sync,qos_profile_sensor_data)
        self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',self.status,qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.land,qos_profile_sensor_data)

    def cloud(self,msg):
        self.cloud_count+=1
        if len(self.data['clouds'])>=12:return
        try:
            points=decode_xyz(msg)
            name=f'cloud_{len(self.data["clouds"]):02d}.npz'
            np.savez_compressed(OUT/name,points=points,stamp=stamp(msg.header.stamp),
                                frame=msg.header.frame_id,received_mono=time.monotonic())
            self.data['clouds'].append({'stamp':stamp(msg.header.stamp),'frame':msg.header.frame_id,
                                      'points_file':name,'valid_points':len(points),'received_mono':time.monotonic()})
        except (ValueError,KeyError,TypeError) as error:self.errors.append(str(error))

    def poses(self,msg):
        if self.frames is None:self.frames=[{'parent':t.header.frame_id,'child':t.child_frame_id} for t in msg.transforms]
        if len(self.data['poses'])>=3000:return
        values=[]
        for t in msg.transforms:
            if any(n in t.child_frame_id for n in ('x500_0','base_link','sando_lidar_link')):
                p=t.transform.translation;q=t.transform.rotation
                values.append({'parent':t.header.frame_id,'child':t.child_frame_id,'stamp':stamp(t.header.stamp),
                               'position':[p.x,p.y,p.z],'quaternion':[q.w,q.x,q.y,q.z]})
        if values:self.data['poses'].append({'received_mono':time.monotonic(),'transforms':values})

    def clock(self,msg):
        if len(self.data['clock'])<3000:self.data['clock'].append({'stamp':stamp(msg.clock),'received_mono':time.monotonic()})

    def odom(self,msg):
        if len(self.data['odometry'])>=4000:return
        self.data['odometry'].append({'timestamp':msg.timestamp,'timestamp_sample':msg.timestamp_sample,
            'position':[float(v) for v in msg.position],'velocity':[float(v) for v in msg.velocity],'q':[float(v) for v in msg.q],
            'pose_frame':msg.pose_frame,'velocity_frame':msg.velocity_frame,
            'received_mono':time.monotonic(),'received_wall':time.time()})

    def sync(self,msg):
        self.data['timesync'].append({'timestamp':msg.timestamp,'remote_timestamp':msg.remote_timestamp,
            'observed_offset':msg.observed_offset,'estimated_offset':msg.estimated_offset,
            'round_trip_time':msg.round_trip_time,'source_protocol':msg.source_protocol,'received_mono':time.monotonic()})

    def status(self,msg):
        self.data['status'].append({'armed':msg.arming_state==VehicleStatus.ARMING_STATE_ARMED,
                                   'nav_state':msg.nav_state,'received_mono':time.monotonic()})

    def land(self,msg):self.data['landed'].append({'landed':msg.landed,'received_mono':time.monotonic()})


def main():
    OUT.mkdir(exist_ok=True)
    rclpy.init();node=Capture()
    try:
        end=time.monotonic()+15
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        data=dict(node.data,frames=node.frames,errors=node.errors,total_clouds_received=node.cloud_count)
        save_json(OUT/'capture.json',data)
        summary={'counts':{k:len(v) for k,v in node.data.items()},'errors':node.errors,
                 'total_clouds_received':node.cloud_count,'control_commands_sent':False}
        save_json(OUT/'summary.json',summary)
        print(json.dumps(summary,indent=2),flush=True)
    except Exception as error:
        save_json(OUT/'failure.json',{'error':str(error),'counts':{k:len(v) for k,v in node.data.items()}})
        raise
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
