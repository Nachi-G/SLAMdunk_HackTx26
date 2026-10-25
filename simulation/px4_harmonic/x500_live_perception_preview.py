"""Mounted X500 LiDAR -> acquisition-time world pose -> calibrated PX4 ENU preview.

Uses named Gazebo poses directly because scene Pose_V lacks ROS TF metadata.
Only a separate preview cloud is published. Never publishes planner or PX4 inputs.
"""
import argparse
from collections import Counter,deque
import json
import math
from pathlib import Path
import queue
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from px4_msgs.msg import VehicleOdometry,TimesyncStatus,VehicleStatus,VehicleLandDetected
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

from perception_geometry import (PoseHistory,pose_matrix,px4_pose_enu,
                                 simulation_sample_time,mean_rigid_transform)
sys.path.insert(0,str(Path(__file__).resolve().parent/'diagnostics'))
from diagnostics.validate_ros_lidar import decode_xyz
from diagnostics.analyze_stationary_lidar import analyze_points

OUT=Path(__file__).resolve().parent/'diagnostics/live_perception'


class LivePerception(Node):
    def __init__(self):
        super().__init__('x500_live_perception_preview')
        self.pose_queue=queue.Queue(maxsize=200)
        self.history=PoseHistory(max_gap=.15,capacity=600)
        self.base=self.sensor=None
        self.pose_received=None
        self.syncs=deque(maxlen=30)
        self.odometry=deque(maxlen=500)
        self.last_matched_odometry=None
        self.reset_counter=None
        self.clouds=deque(maxlen=15)
        self.armed=self.landed=None
        self.status_at=self.land_at=None
        self.calibration_samples=[]
        self.calibration=None
        self.calibration_statistics=None
        self.residuals=[]
        self.published=0
        self.geometry=[]
        self.blocks=Counter()
        self.last_points=None
        self.last_world_points=None
        self.last_stamp=None
        self.last_processed_cloud=-math.inf
        self.fault=None
        self.preview=self.create_publisher(PointCloud2,'/sando_perception/points_map_preview',5)
        self.create_subscription(PointCloud2,'/sando_perception/lidar/points_raw',self.cloud,qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus,'/fmu/out/timesync_status',self.sync,qos_profile_sensor_data)
        self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',self.status,qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.land,qos_profile_sensor_data)
        self.gz=GzNode()
        if not self.gz.subscribe(Pose_V,'/world/default/pose/info',self.raw_pose):
            raise RuntimeError('Could not subscribe to named Gazebo scene poses')
        self.create_timer(.02,self.process)

    def raw_pose(self,msg):
        by_name={p.name:p for p in msg.pose}
        if not all(n in by_name for n in ('x500_0','base_link','sando_lidar_link')):return
        def unpack(p):
            return ([p.position.x,p.position.y,p.position.z],
                    [p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z])
        value=(msg.header.stamp.sec+msg.header.stamp.nsec/1e9,
               unpack(by_name['x500_0']),unpack(by_name['base_link']),
               unpack(by_name['sando_lidar_link']),time.monotonic())
        try:self.pose_queue.put_nowait(value)
        except queue.Full:
            try:self.pose_queue.get_nowait()
            except queue.Empty:pass
            try:self.pose_queue.put_nowait(value)
            except queue.Full:pass

    def status(self,msg):self.armed=msg.arming_state==VehicleStatus.ARMING_STATE_ARMED;self.status_at=time.monotonic()
    def land(self,msg):self.landed=msg.landed;self.land_at=time.monotonic()
    def sync(self,msg):
        if msg.source_protocol==TimesyncStatus.SOURCE_PROTOCOL_DDS:
            self.syncs.append((time.monotonic(),int(msg.estimated_offset)))

    def odom(self,msg):
        if msg.pose_frame!=VehicleOdometry.POSE_FRAME_NED:return
        try:pose=px4_pose_enu(msg.position,msg.q)
        except ValueError:return
        if self.reset_counter is not None and msg.reset_counter!=self.reset_counter:
            self.fault='PX4 estimator reset; preview calibration invalidated'
            return
        self.reset_counter=msg.reset_counter
        self.odometry.append((time.monotonic(),int(msg.timestamp_sample),pose))

    def cloud(self,msg):self.clouds.append((time.monotonic(),msg))

    def process(self):
        now=time.monotonic()
        while True:
            try:stamp,model,base,sensor,received=self.pose_queue.get_nowait()
            except queue.Empty:break
            try:
                base=pose_matrix(*base);sensor=pose_matrix(*sensor)
                extrinsic=np.linalg.inv(base)@sensor
                if not np.allclose(extrinsic[:3,3],[0,0,.3],atol=1e-5):raise ValueError('Sensor mount changed')
                if self.base is not None and (not np.allclose(base,self.base,atol=1e-5) or not np.allclose(sensor,self.sensor,atol=1e-5)):
                    raise ValueError('Model-relative link poses changed unexpectedly')
                if self.history.samples and stamp==self.history.samples[-1][0]:continue
                self.history.add(stamp,*model)
                self.base,self.sensor=base,sensor
                self.pose_received=received
            except ValueError as error:self.fault=str(error)
        if self.fault:return
        if self.pose_received is None or now-self.pose_received>.5:
            self.blocks['No fresh named Gazebo poses']+=1;return
        if not self.syncs or now-self.syncs[-1][0]>2:
            self.blocks['No fresh DDS timesync']+=1;return
        while self.odometry:
            received,stamp,body=self.odometry[0]
            nearest=min(self.syncs,key=lambda s:abs(s[0]-received))
            if abs(nearest[0]-received)>2:self.odometry.popleft();continue
            sample=simulation_sample_time(stamp,nearest[1])
            try:world_body=self.history.at(sample)@self.base
            except ValueError:
                if now-received<.3:break
                self.odometry.popleft();self.blocks['Odometry has no bracketing simulation poses']+=1;continue
            self.odometry.popleft()
            self.last_matched_odometry=received
            if self.calibration is None:
                grounded=(self.armed is False and self.landed is True and self.status_at is not None
                          and now-self.status_at<1 and self.land_at is not None and now-self.land_at<2)
                if not grounded:self.blocks['Calibration requires fresh landed/disarmed state']+=1;continue
                self.calibration_samples.append(body@np.linalg.inv(world_body))
                if len(self.calibration_samples)>=50:
                    try:
                        self.calibration,spread,angle=mean_rigid_transform(self.calibration_samples)
                        self.calibration_statistics={'samples':len(self.calibration_samples),
                                                     'translation_spread_m':spread,'rotation_spread_rad':angle}
                    except ValueError as error:self.fault=str(error);return
            else:
                expected=self.calibration@world_body
                error=float(np.linalg.norm(expected[:3,3]-body[:3,3]))
                self.residuals.append(error)
                angle=math.acos(float(np.clip((np.trace(expected[:3,:3].T@body[:3,:3])-1)/2,-1,1)))
                if error>.25 or angle>.08:
                    self.fault='PX4/world pose alignment changed beyond preview bounds';return
        if self.calibration is None:return
        if self.last_matched_odometry is None or now-self.last_matched_odometry>.3:
            self.blocks['No fresh acquisition-matched odometry']+=1;return
        while self.clouds:
            received,msg=self.clouds[0]
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if now-received>.5:self.clouds.popleft();self.blocks['Cloud queue stale']+=1;continue
            if msg.header.frame_id!='sando_lidar_link':self.clouds.popleft();self.blocks['Unexpected sensor frame']+=1;continue
            if stamp<=self.last_processed_cloud:self.clouds.popleft();self.blocks['Cloud acquisition time reset/duplicate']+=1;continue
            try:world_sensor=self.history.at(stamp)@self.sensor
            except ValueError:
                if now-received<.25:break
                self.clouds.popleft();self.blocks['Cloud has no bracketing poses']+=1;continue
            self.clouds.popleft()
            if self.history.samples[-1][0]-stamp>.4:self.blocks['Cloud is stale in simulation time']+=1;continue
            try:points=decode_xyz(msg)
            except ValueError as error:
                self.blocks[str(error)]+=1;continue
            # Validate actual sensor points against known simulated forest geometry.
            geometry=analyze_points(points,world_sensor)
            if not geometry['status'].startswith('PASS'):
                self.fault='Transformed cloud does not match forest geometry';return
            self.geometry.append(geometry['geometry_match_fraction'])
            world_points=points@world_sensor[:3,:3].T+world_sensor[:3,3]
            base_sensor=np.linalg.inv(self.base)@self.sensor
            body_points=points@base_sensor[:3,:3].T+base_sensor[:3,3]
            self_mask=(np.abs(body_points[:,0])<.6)&(np.abs(body_points[:,1])<.6)&(np.abs(body_points[:,2])<.35)
            world_points=world_points[(~self_mask)&(world_points[:,2]>.15)]
            local=world_points@self.calibration[:3,:3].T+self.calibration[:3,3]
            if not len(local):self.blocks['No above-ground obstacle returns']+=1;continue
            # Downsample without pretending unobserved space is free.
            _,indexes=np.unique(np.floor(local/.15).astype(np.int64),axis=0,return_index=True)
            local=local[indexes]
            header=Header(stamp=msg.header.stamp,frame_id='px4_enu_preview')
            self.preview.publish(point_cloud2.create_cloud_xyz32(header,local.astype(np.float32)))
            self.last_points=local;self.last_world_points=world_points;self.last_stamp=stamp
            self.last_processed_cloud=stamp;self.published+=1

    def report(self):
        good=self.published>=20 and self.calibration is not None and not self.fault and bool(self.residuals)
        return {'status':'PASS: mounted live perception preview' if good else 'NOT PASSED: mounted perception preview',
                'published_clouds':self.published,'fault':self.fault,'startup_or_data_blocks':dict(self.blocks),
                'calibration':self.calibration_statistics,
                'world_to_px4_enu':self.calibration.tolist() if self.calibration is not None else None,
                'maximum_pose_alignment_residual_m':max(self.residuals,default=None),
                'minimum_forest_geometry_match_fraction':min(self.geometry,default=None),
                'last_preview_obstacle_voxels':len(self.last_points) if self.last_points is not None else 0,
                'output_topic':'/sando_perception/points_map_preview','output_frame':'px4_enu_preview',
                'limits':'Stationary grounded calibration/run only. Moving validation, map persistence/unknown space, planner connection and flight controls remain unverified. No SANDO or PX4 input topics published.'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=20);args=parser.parse_args()
    OUT.mkdir(exist_ok=True);rclpy.init();node=LivePerception()
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline and not node.fault:rclpy.spin_once(node,timeout_sec=.1)
        report=node.report();(OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        if node.last_points is not None:
            np.savez_compressed(OUT/'last_cloud.npz',points_px4_enu=node.last_points,points_world=node.last_world_points,
                                stamp_sim=node.last_stamp,world_to_px4_enu=node.calibration)
        print(json.dumps(report,indent=2),flush=True)
        return 0 if report['status'].startswith('PASS') else 1
    finally:
        node.gz.unsubscribe('/world/default/pose/info')
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
