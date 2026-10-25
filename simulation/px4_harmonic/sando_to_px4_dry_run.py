"""Read-only 20 Hz SANDO->NED preview. Creates no ROS publishers."""
import argparse
from collections import Counter,deque
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from dynus_interfaces.msg import DynTraj,State
from px4_msgs.msg import VehicleStatus,VehicleLandDetected,VehicleOdometry
from sando_bridge_core import CubicTrajectory,PreviewWatchdog,yaw_enu_to_ned


class DryRun(Node):
    def __init__(self,trajectory_topic='/trajs',state_topic='/NX01/state'):
        super().__init__('sando_to_px4_read_only_watchdog')
        self.guard=PreviewWatchdog()
        self.position=None
        self.state_received=None
        self.yaw=None
        self.hold_yaw=None
        self.rejections=Counter()
        self.blocks=Counter()
        self.accepted=0
        self.evaluations=0
        self.status=None
        self.status_received=None
        self.landed=None
        self.landed_received=None
        self.last_tick=None
        self.intervals=[]
        self.last_output=None
        self.last_alignment=None
        self.alignment_samples=[]
        self.odometry_samples=deque(maxlen=100)
        self.bridge_comparison=None
        self.start=time.monotonic()
        self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.odometry,qos_profile_sensor_data)
        self.create_subscription(State,state_topic,self.state,qos_profile_sensor_data)
        self.create_subscription(DynTraj,trajectory_topic,self.trajectory,10)
        self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',self.vehicle_status,qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.land,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
        print('READ ONLY: no publishers, mode changes, arming, or flight commands.',flush=True)

    def odometry(self,msg):
        if msg.pose_frame!=VehicleOdometry.POSE_FRAME_NED:
            return
        p=np.asarray([msg.position[1],msg.position[0],-msg.position[2]])
        if np.isfinite(p).all():self.odometry_samples.append((time.monotonic(),p))

    def state(self,msg):
        p=np.asarray([msg.pos.x,msg.pos.y,msg.pos.z])
        q=np.asarray([msg.quat.w,msg.quat.x,msg.quat.y,msg.quat.z])
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if msg.header.frame_id!='map' or not np.isfinite(p).all() or not np.isfinite(q).all():
            return
        norm=np.linalg.norm(q)
        if not .9<norm<1.1 or not -.1<=time.time()-stamp<=.5:
            return
        w,x,y,z=q/norm
        self.yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        self.position=p
        self.state_received=time.monotonic()
        if self.odometry_samples:
            received,position=min(self.odometry_samples,key=lambda item:abs(item[0]-self.state_received))
            self.bridge_comparison={
                'px4_position_converted_to_enu_m':position.tolist(),
                'sando_state_position_enu_m':p.tolist(),
                'difference_m':float(np.linalg.norm(p-position)),
                'receipt_time_separation_s':abs(received-self.state_received),
                'limits':'Receipt-nearest position comparison only; not acquisition-time synchronization.'}

    def vehicle_status(self,msg):
        self.status={'armed':msg.arming_state==VehicleStatus.ARMING_STATE_ARMED,
                     'arming_state':msg.arming_state,'nav_state':msg.nav_state,
                     'offboard':msg.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                     'failsafe':msg.failsafe}
        self.status_received=time.monotonic()

    def land(self,msg):
        self.landed=msg.landed
        self.landed_received=time.monotonic()

    def trajectory(self,msg):
        if msg.id!=1 or not msg.is_agent:
            return
        now=time.monotonic()
        try:
            if self.state_received is None or now-self.state_received>.3:
                raise ValueError('No recent valid state when trajectory arrived')
            candidate=CubicTrajectory.from_message(msg)
            wall=time.time()
            if candidate.times[0]<=wall<candidate.times[-1]:
                reference=candidate.evaluate(wall)
                self.last_alignment={
                    'measured_position_enu_m':self.position.tolist(),
                    'trajectory_position_enu_m':list(reference.position),
                    'position_difference_m':float(np.linalg.norm(np.array(reference.position)-self.position)),
                    'state_receipt_age_s':time.monotonic()-self.state_received,
                    'trajectory_start_age_s':wall-candidate.times[0],
                    'trajectory_remaining_s':candidate.times[-1]-wall,
                    'planner_reported_state_enu_m':[msg.pos.x,msg.pos.y,msg.pos.z],
                    'planner_state_difference_m':float(np.linalg.norm(np.array([msg.pos.x,msg.pos.y,msg.pos.z])-self.position)),
                    'trajectory_initial_position_enu_m':list(candidate.segment(0,0).position),
                    'trajectory_header_frame':msg.header.frame_id,
                    'header_age_s':wall-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9}
                if len(self.alignment_samples)<10:
                    self.alignment_samples.append(dict(self.last_alignment,
                        times=list(candidate.times),coefficients=candidate.coeff.tolist()))
            self.guard.accept(candidate,wall,time.monotonic(),self.position)
            self.accepted+=1
            if self.hold_yaw is None:
                self.hold_yaw=yaw_enu_to_ned(self.yaw)
        except ValueError as error:
            self.guard.block(str(error))
            self.rejections[str(error)]+=1

    def tick(self):
        mono=time.monotonic()
        if self.last_tick is not None:self.intervals.append(mono-self.last_tick)
        self.last_tick=mono
        try:
            position=self.position if self.position is not None else np.array([math.nan]*3)
            age=mono-self.state_received if self.state_received is not None else math.inf
            point=self.guard.tick(time.time(),mono,position,age).ned()
            self.last_output={'position_ned_m':point.position,'velocity_ned_m_s':point.velocity,
                              'acceleration_ned_m_s2':point.acceleration,'jerk_ned_m_s3':point.jerk,
                              'held_yaw_ned_rad':self.hold_yaw,
                              'timestamp_candidate_us':time.time_ns()//1000,
                              'timestamp_sync_validated':False}
            self.evaluations+=1
        except ValueError as error:
            self.blocks[str(error)]+=1

    def report(self):
        mono=time.monotonic()
        return {'status':'READ-ONLY OBSERVATION; not flight approval',
                'duration_wall_s':mono-self.start,'validated_trajectories':self.accepted,
                'valid_20hz_evaluations':self.evaluations,'rejected_trajectories':dict(self.rejections),
                'watchdog_blocks':dict(self.blocks),'last_valid_preview':self.last_output,
                'last_alignment_observation':self.last_alignment,
                'alignment_samples':self.alignment_samples,
                'state_bridge_comparison':self.bridge_comparison,
                'maximum_timer_interval_s':max(self.intervals,default=None),
                'px4_status':self.status,
                'px4_status_age_s':mono-self.status_received if self.status_received is not None else None,
                'landed':self.landed,
                'landed_status_age_s':mono-self.landed_received if self.landed_received is not None else None,
                'limits':'No collision validation in this live preview. State bridge stamps receipt time; PX4/sensor time and frame alignment remain unverified. Watchdog blocks only suppress preview; no flight failsafe action is implemented.'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--output',default='diagnostics/live_bridge_preview.json')
    parser.add_argument('--trajectory-topic',default='/trajs')
    parser.add_argument('--state-topic',default='/NX01/state')
    args=parser.parse_args()
    rclpy.init()
    node=DryRun(args.trajectory_topic,args.state_topic)
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        report=node.report()
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
