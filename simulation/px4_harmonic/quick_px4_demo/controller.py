"""Separate ground-truth moving-obstacle demo. PX4 flies; never teleports the drone.

Default is preview (no PX4 command publishers). --fly requires explicit user approval.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import threading
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (VehicleOdometry,VehicleStatus,VehicleLandDetected,VehicleCommand,BatteryStatus,
                          VehicleCommandAck,OffboardControlMode,TrajectorySetpoint,TimesyncStatus)
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
from perception_geometry import pose_matrix,px4_pose_enu,mean_rigid_transform
from quick_px4_demo.planner import trees,route,resolve_pin,movers,DRONE_RADIUS,MARGIN,choose_velocity,clearance,MAX_SPEED,SLALOM_WAYPOINTS,CHALLENGE_WAYPOINTS
from quick_px4_demo import obstacles
from quick_px4_demo.landing_recovery import recovery_target,recovery_velocity,swept_distances,landing_site,landing_segment_clear
from quick_px4_demo.readiness import ground_readiness,simulation_frame_candidate,feedback_timestamp

OUT=ROOT/'diagnostics/quick_px4_demo'


class Demo(Node):
    def __init__(self,fly,profile='baseline',manual_control=False,wait_for_start=False):
        super().__init__('quick_px4_ground_truth_demo')
        self.wait_for_start=wait_for_start;self.start_requested=False;self.run_started=None;self.home_world=None
        self.fly=fly;self.profile=profile;self.gz=GzNode();self.stop_thread=threading.Event()
        self.lock=threading.Lock();self.names=set();self.world_body=None;self.sim_time=None;self.pose_at=None
        self.last_pose_names=[]
        self.odom=None;self.odom_at=None;self.status=None;self.status_at=None;self.landed=None;self.land_at=None
        self.sync_at=None;self.reset=None;self.calibration=None;self.calibration_samples=[]
        self.phase='PREFLIGHT';self.phase_at=time.monotonic();self.started=self.phase_at
        self.movers_ready=False;self.worker_error=None;self.motion_start=None;self.path=None;self.path_time=-100.;self.path_goal_index=-1
        self.mission=list(CHALLENGE_WAYPOINTS) if profile=='challenge' else (list(SLALOM_WAYPOINTS) if profile=='slalom' else [(12.,0.)])
        self.waypoint_index=0;self.completed_waypoints=[]
        self.static=trees();self.goal=np.asarray(self.mission[0]);self.command=np.zeros(2);self.next_velocity=np.zeros(2)
        self.plan_valid=False;self.plan_at=None;self.plan_blocks=0;self.last_plan_sim=None
        self.minimum_clearance=math.inf;self.goal_reached=False;self.owns_flight=False;self.arming_requested=False;self.offboard_requested=False
        self.takeoff_target=None;self.yaw=None;self.last_tick=None;self.maximum_timer_interval=0.;self.command_at=-100.;self.landing_at=None
        self.events=[];self.acknowledgements=[];self.tick_count=0;self.counters=Counter();self.hold_since=None
        self.failure=None
        self.alignment_samples=[];self.last_alignment=None;self.progress_at=0.
        self.obstacle_rpc=None
        self.plan_revision=0;self.prediction=np.empty((0,2));self.prediction_at=-100.;self.plan_output_at=-100.
        self.recovery_goal=None;self.recovery_since=None;self.landing_target=None
        self.flight_started=None;self.flight_elapsed=0.;self.battery=None;self.battery_at=None
        self.manual=None;self.command_server=None;self.was_manual=False;self.manual_used=False
        self.collision=None;self.manual_rejoin=False;self.rejoin_at=None
        if manual_control:
            if not fly:raise ValueError('--manual-control requires --fly')
            from quick_px4_demo.manual import ManualSteering
            from slam_dunk_bridge.command_server import CommandServer
            self.manual=ManualSteering(navigate=self.navigate,navigation_ready=self.destination_ready,mission_command=self.mission_command,mission_status=self.mission_status);self.command_server=CommandServer(self.manual)
            self.command_server.start()
            if not self.gz.subscribe(Contacts,'/demo/contacts',self.on_contacts):raise RuntimeError('Contact subscription failed')
        self.control=self.setpoints=self.commands=None
        if fly:
            self.control=self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',1)
            self.setpoints=self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',1)
            self.commands=self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
        self.create_subscription(BatteryStatus,'/fmu/out/battery_status_v1',self.on_battery,qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry,'/fmu/out/vehicle_odometry',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',self.on_status,qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.on_land,qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus,'/fmu/out/timesync_status',self.on_sync,qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck,'/fmu/out/vehicle_command_ack_v1',self.on_ack,qos_profile_sensor_data)
        if not self.gz.subscribe(Pose_V,'/world/default/pose/info',self.on_pose):raise RuntimeError('Gazebo pose subscription failed')
        self.worker=threading.Thread(target=self.work,daemon=True);self.worker.start()
        self.create_timer(.05,self.tick)

    def event(self,text):
        self.events.append({'wall_s':time.monotonic()-self.started,'event':text})
        print(text,flush=True)
        if self.manual is not None:
            state={'phase':self.phase,'event':text,'collision':self.collision,'failure':self.failure,'updated_wall_s':time.time()}
            temp=OUT/'manual_state.tmp';temp.write_text(json.dumps(state));temp.replace(OUT/'manual_state.json')

    def transition(self,phase):
        self.phase=phase;self.phase_at=time.monotonic()
        if phase=='TAKEOFF':self.flight_started=self.phase_at
        if phase in ('DONE','ABORT') and self.flight_started is not None:self.flight_elapsed=self.phase_at-self.flight_started
        self.event(phase)

    def on_battery(self,message):
        if message.connected and math.isfinite(message.remaining) and 0<=message.remaining<=1:
            self.battery=100*float(message.remaining);self.battery_at=time.monotonic()

    def mission_status(self):
        now=time.monotonic()
        fresh=(self.odom_at is not None and now-self.odom_at<.3 and self.pose_at is not None and now-self.pose_at<.3
               and self.status_at is not None and now-self.status_at<1 and self.status is not None and not self.status.gcs_connection_lost and not self.status.failsafe)
        grounded=(self.landed is True and self.land_at is not None and now-self.land_at<2 and self.status is not None
                  and self.status.arming_state==VehicleStatus.ARMING_STATE_DISARMED)
        start_ready=bool(self.wait_for_start and fresh and grounded and self.failure is None and not self.start_requested
                         and (self.phase=='DONE' or (self.phase=='PREFLIGHT' and self.calibration is not None and self.plan_valid)))
        if not self.wait_for_start:reason='Use start_dashboard_controller.sh to enable dashboard Start'
        elif self.failure:reason='Restart controller after resolving: '+self.failure
        elif self.status is None or not fresh:reason='Waiting for fresh PX4/Gazebo telemetry and QGC connection'
        elif not grounded:reason='Waiting for drone to land and disarm'
        elif self.start_requested:reason='Start already requested'
        elif self.phase not in ('PREFLIGHT','DONE'):reason='Flight already active: '+self.phase
        elif self.calibration is None:reason='Calibrating takeoff frame'
        elif not self.plan_valid and self.phase!='DONE':reason='Waiting for a valid obstacle-checked route'
        else:reason='Ready — take off and slowly clear nearby obstacle before course' if self.recovery_goal is not None else 'Ready — click Start'
        return {'start_reason':reason,'start_ready':start_ready,'return_ready':bool(fresh and self.failure is None and self.home_world is not None and self.phase in ('TAKEOFF','NAVIGATE')),
                'mission_phase':self.phase,'home_position':self.home_world.tolist() if self.home_world is not None else None}

    def mission_command(self,action):
        flags=self.mission_status()
        if action=='start':
            if not flags['start_ready']:raise ValueError('Start requires ready standby, QGC connected, and landed/disarmed drone')
            self.start_requested=True
            return {'reason':'Start accepted — takeoff and autonomous course requested'}
        if action=='return_home':
            if not flags['return_ready']:raise ValueError('Return Home is available during takeoff/navigation with fresh telemetry')
            try:details=self.navigate(self.home_world[:2].tolist(),allow_snap=False,for_landing=True)
            except ValueError as error:raise ValueError('Return Home route unavailable: '+str(error)) from error
            details['reason']='Returning to a clear landing site near takeoff' if details.get('adjusted') else 'Returning to takeoff position with obstacle avoidance; landing on arrival'
            return details
        raise ValueError('Unknown mission command')

    def reset_completed_run(self):
        self.mission=list(CHALLENGE_WAYPOINTS) if self.profile=='challenge' else (list(SLALOM_WAYPOINTS) if self.profile=='slalom' else [(12.,0.)])
        self.waypoint_index=0;self.completed_waypoints=[];self.goal=np.asarray(self.mission[0]);self.goal_reached=False
        self.calibration=None;self.calibration_samples=[];self.takeoff_target=None;self.home_world=None
        self.owns_flight=False;self.arming_requested=False;self.offboard_requested=False;self.hold_since=None
        self.was_manual=False;self.manual_rejoin=False;self.plan_valid=False;self.path=None;self.path_goal_index=-1;self.plan_revision+=1
        self.command=np.zeros(2);self.next_velocity=np.zeros(2);self.prediction=np.empty((0,2))
        self.recovery_goal=None;self.recovery_since=None;self.landing_target=None
        self.flight_started=None;self.flight_elapsed=0.;self.started=time.monotonic();self.transition('PREFLIGHT')

    def destination_ready(self):
        now=time.monotonic()
        return (self.phase in ('PREFLIGHT','WARMUP','REQUEST_OFFBOARD','ARMING','TAKEOFF','NAVIGATE')
                and self.failure is None and self.calibration is not None and self.movers_ready
                and self.odom_at is not None and now-self.odom_at<.3
                and self.pose_at is not None and now-self.pose_at<.3
                and self.status_at is not None and now-self.status_at<1
                and self.status is not None and not self.status.gcs_connection_lost)

    def navigate(self,xy,allow_snap=True,for_landing=False):
        if self.phase not in ('PREFLIGHT','WARMUP','REQUEST_OFFBOARD','ARMING','TAKEOFF','NAVIGATE') or self.failure is not None:raise ValueError('Flight must be preparing or navigating')
        position=self.world_body[:2,3].copy();elapsed=self.sim_time-self.motion_start
        try:
            if for_landing:resolved,new_path=landing_site(position,np.asarray(xy),elapsed,self.static,self.profile);adjusted=bool(np.linalg.norm(resolved-np.asarray(xy))>.05)
            elif allow_snap:resolved,new_path,adjusted=resolve_pin(position,np.asarray(xy),elapsed,self.static,profile=self.profile)
            else:resolved=np.asarray(xy,dtype=float);new_path=route(position,resolved,elapsed,self.static,profile=self.profile);adjusted=False
        except ValueError as error:raise ValueError('Pin rejected: '+str(error)) from error
        with self.lock:
            self.plan_revision+=1;self.mission=[tuple(map(float,resolved))];self.waypoint_index=0
            self.goal=np.asarray(resolved,dtype=float);self.path=new_path;self.path_goal_index=0
            self.manual_rejoin=self.phase=='NAVIGATE';self.rejoin_at=time.monotonic();self.plan_valid=False
            if self.phase=='NAVIGATE':self.phase_at=time.monotonic()
            self.prediction=np.empty((0,2));self.goal_reached=False;self.landing_target=None;self.recovery_goal=None;self.recovery_since=None
        distance=float(np.linalg.norm(resolved-np.asarray(xy)))
        reason=(f'Pin adjusted {distance:.2f} m to nearby clear destination' if adjusted else 'Pinned destination accepted')
        self.event('PIN DESTINATION: requested='+str(xy)+' accepted='+str(resolved.tolist()))
        return {'position':resolved.tolist(),'requested_position':list(xy),'adjusted':adjusted,'reason':reason}
    def on_status(self,m):self.status=m;self.status_at=time.monotonic()
    def on_land(self,m):self.landed=m.landed;self.land_at=time.monotonic()

    def on_sync(self,m):
        if m.source_protocol==TimesyncStatus.SOURCE_PROTOCOL_DDS:self.sync_at=time.monotonic()

    def on_odom(self,m):
        if m.pose_frame!=VehicleOdometry.POSE_FRAME_NED or m.velocity_frame!=VehicleOdometry.VELOCITY_FRAME_NED:return
        if not np.isfinite(np.r_[m.position,m.velocity,m.q]).all():return
        if self.reset is not None and m.reset_counter!=self.reset:self.worker_error='PX4 estimator reset'
        self.reset=m.reset_counter;self.odom=m;self.odom_at=time.monotonic()

    def on_ack(self,m):
        if self.fly and m.command in (VehicleCommand.VEHICLE_CMD_DO_SET_MODE,VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,VehicleCommand.VEHICLE_CMD_NAV_LAND):
            self.acknowledgements.append({'command':m.command,'result':m.result})
            if m.result not in (VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS):
                self.worker_error=f'PX4 command {m.command} rejected: result {m.result}'

    def on_pose(self,m):
        by_name={p.name:p for p in m.pose}
        self.last_pose_names=[n for n in by_name if 'x500' in n or 'base_link' in n]
        if 'x500_0' not in by_name or 'base_link' not in by_name:return
        def matrix(p):return pose_matrix([p.position.x,p.position.y,p.position.z],
                                        [p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z])
        stamp=m.header.stamp.sec+m.header.stamp.nsec/1e9
        with self.lock:
            if self.sim_time is not None and stamp<self.sim_time:self.worker_error='Gazebo clock reset';return
            self.world_body=matrix(by_name['x500_0'])@matrix(by_name['base_link'])
            self.sim_time=stamp;self.pose_at=time.monotonic();self.names=set(by_name)

    def on_contacts(self,message):
        if self.phase!='NAVIGATE':return
        for contact in message.contact:
            a,b=contact.collision1.name,contact.collision2.name
            if 'x500_0' in a or 'x500_0' in b:
                self.collision={'collision1':a,'collision2':b,'sim_time_s':self.sim_time}
                return

    def work(self):
        try:
            while not self.stop_thread.is_set():
                now=time.monotonic()
                ready=(self.world_body is not None and self.status is not None and self.status_at is not None
                       and now-self.status_at<1 and self.status.arming_state==VehicleStatus.ARMING_STATE_DISARMED
                       and self.landed is True and self.land_at is not None and now-self.land_at<2)
                if ready:break
                time.sleep(.02)
            if self.stop_thread.is_set():return
            self.obstacle_rpc=obstacles.ObstacleRPC(self.profile)
            self.obstacle_rpc.create(lambda:self.names)
            self.motion_start=self.sim_time
            self.movers_ready=True
            while not self.stop_thread.is_set():
                if self.phase not in ('LANDING','DONE','ABORT'):
                    elapsed=self.sim_time-self.motion_start
                    self.obstacle_rpc.move(elapsed)
                    if self.manual is not None and self.manual.velocity() is not None:
                        time.sleep(.05);continue
                    if self.calibration is not None and self.odom is not None:
                        position=self.world_body[:2,3].copy()
                        v_enu=np.array([self.odom.velocity[1],self.odom.velocity[0],-self.odom.velocity[2]])
                        v_world=self.calibration[:3,:3].T@v_enu
                        with self.lock:
                            target=self.goal.copy();goal_index=self.waypoint_index;revision=self.plan_revision
                        dt=.1 if self.last_plan_sim is None else elapsed-self.last_plan_sim
                        self.last_plan_sim=elapsed
                        velocity=v_world[:2] if self.phase=='NAVIGATE' else np.zeros(2)
                        if self.recovery_goal is not None and swept_distances(position,self.static,self.profile).min()>=.70:
                            self.recovery_goal=None;self.recovery_since=None;self.path=None
                            print('RECOVERY COMPLETE: normal collision-checked navigation resumed',flush=True)
                        valid=False;prediction=np.empty((0,2));next_command=np.zeros(2)
                        if self.recovery_goal is None:
                            if self.path is None or self.path_goal_index!=goal_index or (self.profile=='baseline' and elapsed-self.path_time>1.5):
                                try:
                                    candidate_path=route(position,target,elapsed,self.static,profile=self.profile)
                                    with self.lock:
                                        if revision!=self.plan_revision:continue
                                        self.path=candidate_path;self.path_time=elapsed;self.path_goal_index=goal_index
                                except ValueError:self.path=None
                            if self.path is not None:
                                next_command,valid,_,prediction=choose_velocity(position,velocity,target,self.path,elapsed,self.static,self.command,dt,profile=self.profile,return_prediction=True)
                            if not valid:
                                try:
                                    escape,escape_path=recovery_target(position,target,elapsed,self.static,self.profile)
                                    if revision!=self.plan_revision:continue
                                    self.recovery_goal=escape;self.recovery_since=elapsed
                                    self.path=escape_path;self.path_goal_index=goal_index
                                    print('RECOVERY: checked low-speed escape from obstacle buffer to '+str(escape.tolist()),flush=True)
                                except ValueError:pass
                        if self.recovery_goal is not None:
                            next_command,valid,prediction=recovery_velocity(position,velocity,self.recovery_goal,self.command,dt,self.static,self.profile)
                            if self.recovery_since is not None and elapsed-self.recovery_since>25 and self.phase=='NAVIGATE':valid=False
                        with self.lock:
                            if revision!=self.plan_revision:continue
                            self.plan_valid=valid;self.command=next_command if self.phase=='NAVIGATE' else np.zeros(2)
                            self.prediction=prediction;self.prediction_at=time.monotonic()
                        if self.manual_rejoin and self.plan_valid:self.manual_rejoin=False
                        self.next_velocity=self.command.copy();self.plan_at=time.monotonic()
                        if not self.plan_valid:self.plan_blocks+=1
                time.sleep(.05)
        except Exception as error:self.worker_error=str(error)

    def vehicle_command(self,command,p1=0.,p2=0.):
        if not self.fly:return
        m=VehicleCommand();m.timestamp=int(self.odom.timestamp) if self.odom is not None else 0;m.command=command
        m.param1=float(p1);m.param2=float(p2);m.target_system=1;m.target_component=1
        m.source_system=1;m.source_component=191;m.from_external=True
        self.commands.publish(m);self.command_at=time.monotonic()

    def stream(self,position=None,velocity=None):
        if not self.fly:return
        now=feedback_timestamp(int(self.odom.timestamp),self.odom_at,time.monotonic())
        mode=OffboardControlMode();mode.timestamp=now;mode.position=position is not None;mode.velocity=position is None
        target=TrajectorySetpoint();target.timestamp=now
        target.position=list(map(float,position)) if position is not None else [math.nan]*3
        target.velocity=list(map(float,velocity)) if velocity is not None else [math.nan]*3
        target.acceleration=[math.nan]*3;target.jerk=[math.nan]*3;target.yaw=float(self.yaw);target.yawspeed=0.
        self.setpoints.publish(target);self.control.publish(mode)

    def fail(self,reason):
        if self.failure is None:self.failure=reason;self.event('STOP: '+reason)
        if self.fly and (self.owns_flight or self.arming_requested) and self.status is not None and self.status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.phase!='LANDING':self.transition('LANDING')
            if time.monotonic()-self.command_at>1:self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        else:self.transition('ABORT')

    def publish_dashboard_plan(self):
        now=time.monotonic()
        if now-self.plan_output_at<.2:return
        self.plan_output_at=now
        active=self.phase=='NAVIGATE' and self.failure is None
        manual_active=self.manual is not None and self.manual.velocity() is not None
        z=float(self.world_body[2,3]) if self.world_body is not None else 2.
        with self.lock:
            path=self.path.copy() if active and not manual_active and self.path is not None else np.empty((0,2))
            predicted=self.prediction.copy() if active and not manual_active and now-self.prediction_at<.5 else np.empty((0,2))
            goal=self.goal.tolist()
        elapsed=(now-self.flight_started if self.flight_started is not None and self.phase not in ('DONE','ABORT') else self.flight_elapsed)
        measured=self.battery is not None and self.battery_at is not None and now-self.battery_at<2
        message={'frame_id':'gazebo_world','updated_monotonic_s':now,'phase':self.phase,
                 'route':[[float(x),float(y),z] for x,y in path[:2000]],
                 'prediction':[[float(x),float(y),z] for x,y in predicted],
                 'target':[float(goal[0]),float(goal[1]),z] if active else None,
                 'bounds':[-2,16,-10,10],
                 'keepouts':[[float(x),float(y),float(radius+DRONE_RADIUS+MARGIN+.15),0.] for x,y,radius in self.static]+
                            [[float(x),float(y),float(radius+DRONE_RADIUS+MARGIN+.3),float(amplitude)] for x,y,amplitude,omega,phase,radius in movers(self.profile)],
                 'prediction_valid':bool(active and not manual_active and self.plan_valid and len(predicted)),
                 'manual':manual_active,'flight_seconds':max(0.,elapsed),'buffer_recovery':self.recovery_goal is not None,'landing_target':self.landing_target.tolist() if self.landing_target is not None else None,
                 'velocity_world':(self.calibration[:3,:3].T@np.array([self.odom.velocity[1],self.odom.velocity[0],-self.odom.velocity[2]])).tolist() if self.calibration is not None and self.odom is not None and self.odom_at is not None and now-self.odom_at<.3 else None,
                 'battery':self.battery if measured else 100.,'battery_source':'px4_simulated' if measured else 'placeholder'}
        temp=OUT/'live_plan.tmp';temp.write_text(json.dumps(message,allow_nan=False));temp.replace(OUT/'live_plan.json')

    def tick(self):
        try:self.control_tick()
        finally:
            try:self.publish_dashboard_plan()
            except (OSError,ValueError):pass

    def control_tick(self):
        now=time.monotonic();self.tick_count+=1
        dt=.05 if self.last_tick is None else now-self.last_tick;self.last_tick=now
        self.maximum_timer_interval=max(self.maximum_timer_interval,dt)
        if self.manual is not None:
            position=self.world_body[:3,3] if self.world_body is not None else [0,0,0]
            yaw=math.atan2(self.world_body[1,0],self.world_body[0,0]) if self.world_body is not None else 0.
            available=(self.phase=='NAVIGATE' and self.failure is None and self.odom_at is not None
                       and now-self.odom_at<.3 and self.pose_at is not None and now-self.pose_at<.3
                       and self.status is not None and self.status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            self.manual.observe(position,yaw,available)
        if self.phase=='DONE' and self.wait_for_start and self.start_requested and self.failure is None:self.reset_completed_run()
        if self.phase in ('DONE','ABORT'):return
        if self.collision is not None and self.failure is None:
            if self.manual is not None:self.manual.stop('COLLISION DETECTED — mission failed; landing requested')
            self.fail('Gazebo contact confirmed: '+str(self.collision));return
        if self.phase=='LANDING':
            if self.landed is True and self.status and self.status.arming_state==VehicleStatus.ARMING_STATE_DISARMED:
                self.transition('DONE');return
            if self.status and self.status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if self.odom_at is not None and now-self.odom_at<=.3:
                    self.stream(velocity=np.zeros(3))
                if now-self.command_at>1:self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if now-self.phase_at>40:self.failure=self.failure or 'Landing/disarming confirmation timed out';self.transition('ABORT')
            return
        if self.command_server is not None and self.command_server.error:self.fail('Manual control server failed');return
        if self.worker_error:self.fail(self.worker_error);return
        fresh=(self.odom_at is not None and now-self.odom_at<.3 and self.pose_at is not None
               and now-self.pose_at<.3 and self.status_at is not None and now-self.status_at<1)
        if not fresh:
            if self.owns_flight:self.fail('State, pose, or status stale')
            elif now-self.started>20 and not self.wait_for_start:self.fail('Required telemetry unavailable')
            return
        if self.manual is not None and 'demo_solid_contact_world' not in self.names:
            self.fail('Manual collision tests require scripts/start_solid_forest.sh');return
        if self.status.failsafe:self.fail('PX4 reports failsafe');return
        if self.fly and self.status.gcs_connection_lost:
            if self.owns_flight or (now-self.started>20 and not self.wait_for_start):self.fail('QGroundControl/GCS heartbeat missing')
            return
        if self.owns_flight and self.status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.failure='PX4 left Offboard; yielding to selected mode';self.transition('ABORT');return
        if dt>.2 and self.owns_flight:self.fail('Control timer stalled');return
        if self.sync_at is None or now-self.sync_at>2:
            if self.owns_flight:self.fail('DDS synchronization updates missing')
            return
        if self.calibration is None:
            readiness=ground_readiness(self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED,
                                       self.landed,now-self.status_at,
                                       now-self.land_at if self.land_at is not None else None)
            if readiness=='WAIT':
                if now-self.started>20:self.fail('Timed out waiting for fresh landed/disarmed confirmation')
                return
            if readiness=='REJECT':
                self.fail('Demo must start landed and disarmed');return
            self.calibration_samples.append(simulation_frame_candidate(self.world_body,px4_pose_enu(self.odom.position,self.odom.q)))
            if len(self.calibration_samples)<40:return
            try:self.calibration,_,_=mean_rigid_transform(self.calibration_samples)
            except ValueError as error:self.fail(str(error));return
            self.home_world=self.world_body[:3,3].copy()
            self.takeoff_target=np.array(self.odom.position,dtype=float);self.takeoff_target[2]-=2.
            w,x,y,z=map(float,self.odom.q);self.yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            self.event('Ground-truth frame calibration ready')
        expected=self.calibration@self.world_body
        measured=px4_pose_enu(self.odom.position,self.odom.q)
        error_vector=expected[:3,3]-measured[:3,3]
        self.last_alignment={'wall_s':now-self.started,'sim_time_s':self.sim_time,
            'phase':self.phase,'world_position':self.world_body[:3,3].tolist(),
            'measured_px4_enu_position':measured[:3,3].tolist(),
            'predicted_px4_enu_position':expected[:3,3].tolist(),
            'error_vector_m':error_vector.tolist(),'error_norm_m':float(np.linalg.norm(error_vector)),
            'px4_feedback_timestamp_us':int(self.odom.timestamp),
            'wall_minus_px4_timestamp_s':time.time()-self.odom.timestamp/1e6,
            'timesync_receipt_age_s':now-self.sync_at if self.sync_at is not None else None,
            'px4_quaternion_ned_frd':list(map(float,self.odom.q)),
            'world_body_rotation':self.world_body[:3,:3].tolist()}
        if len(self.alignment_samples)<10000:self.alignment_samples.append(self.last_alignment)
        if self.last_alignment['error_norm_m']>.25:self.fail('PX4/Gazebo position alignment diverged');return
        if self.fly and now-self.progress_at>1:
            self.progress_at=now
            print(f'{self.phase}: world XY=({self.world_body[0,3]:.2f}, {self.world_body[1,3]:.2f}), '
                  f'height NED={self.odom.position[2]:.2f}, frame error={self.last_alignment["error_norm_m"]:.3f} m',flush=True)
        if not self.movers_ready or self.plan_at is None:return
        if not self.fly:
            if self.phase!='PREVIEW':self.transition('PREVIEW')
            return
        if self.phase=='PREFLIGHT':
            if self.wait_for_start and not self.start_requested:return
            conflicts=[p for p in self.get_publishers_info_by_topic('/fmu/in/trajectory_setpoint') if p.node_name!=self.get_name()]
            if conflicts:self.fail('Another trajectory-setpoint publisher exists');return
            self.start_requested=False;self.run_started=now;self.transition('WARMUP')
        if self.phase=='WARMUP':
            self.stream(position=np.array(self.odom.position,dtype=float))
            if now-self.phase_at>2:self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1,6);self.offboard_requested=True;self.transition('REQUEST_OFFBOARD')
        elif self.phase=='REQUEST_OFFBOARD':
            self.stream(position=np.array(self.odom.position,dtype=float))
            if self.status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.arming_requested=True;self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1);self.transition('ARMING')
            elif now-self.phase_at>5:self.fail('PX4 did not enter Offboard')
        elif self.phase=='ARMING':
            self.stream(position=np.array(self.odom.position,dtype=float))
            if self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED:self.owns_flight=True;self.transition('TAKEOFF')
            elif now-self.phase_at>5:self.fail('PX4 did not arm')
        elif self.phase=='TAKEOFF':
            self.stream(position=self.takeoff_target)
            if abs(self.odom.position[2]-self.takeoff_target[2])<.15 and abs(self.odom.velocity[2])<.2:
                if self.hold_since is None:self.hold_since=now
                elif now-self.hold_since>1:self.hold_since=None;self.transition('NAVIGATE');self.recovery_since=self.sim_time-self.motion_start if self.recovery_goal is not None else None
            else:self.hold_since=None
            if now-self.phase_at>25:self.fail('Takeoff timed out')
        elif self.phase=='NAVIGATE':
            manual_velocity=self.manual.velocity() if self.manual is not None else None
            if manual_velocity is not None:
                if not self.was_manual:
                    self.recovery_goal=None;self.recovery_since=None
                    self.event('MANUAL: obstacle avoidance bypassed while held')
                self.was_manual=True;self.manual_used=True
                v_map=self.calibration[:3,:3]@np.r_[manual_velocity,0.]
                self.stream(velocity=np.array([v_map[1],v_map[0],np.clip(.7*(self.takeoff_target[2]-self.odom.position[2]),-.35,.35)]))
                return
            if self.was_manual:
                self.was_manual=False;self.manual_rejoin=True;self.rejoin_at=now;self.prediction=np.empty((0,2))
                with self.lock:
                    self.plan_revision+=1;self.path=None;self.path_goal_index=-1;self.command=np.array([self.odom.velocity[1],self.odom.velocity[0]])
                self.event('MANUAL RELEASED: braking and replanning to current checkpoint')
                self.stream(velocity=np.zeros(3));return
            if self.manual_rejoin:
                self.stream(velocity=np.zeros(3))
                if now-self.rejoin_at>3:self.fail('No safe route to rejoin after manual steering')
                return
            if now-self.plan_at>.4 or not self.plan_valid:
                self.stream(velocity=np.zeros(3))
                self.fail('No fresh collision-checked velocity');return
            elapsed=self.sim_time-self.motion_start
            separation=float(clearance(self.world_body[:2,3],elapsed,self.static,margin=0,profile=self.profile))
            self.minimum_clearance=min(self.minimum_clearance,separation)
            if separation<.25:self.fail('Measured obstacle clearance below 0.25 m');return
            v_world=np.r_[self.next_velocity,0.]
            v_map=self.calibration[:3,:3]@v_world
            v_ned=np.array([v_map[1],v_map[0],np.clip(.7*(self.takeoff_target[2]-self.odom.position[2]),-.35,.35)])
            self.stream(velocity=v_ned)
            with self.lock:
                if np.linalg.norm(self.world_body[:2,3]-self.goal)<(.4 if self.profile!='baseline' else .5):
                    self.completed_waypoints.append({'index':self.waypoint_index,
                        'target_xy':self.goal.tolist(),'actual_xy':self.world_body[:2,3].tolist(),
                        'wall_s':now-self.started})
                    if self.waypoint_index+1<len(self.mission):
                        self.plan_revision+=1;self.waypoint_index+=1;self.goal=np.asarray(self.mission[self.waypoint_index])
                        self.path=None;self.path_time=-100.;self.path_goal_index=-1
                        self.event(f'SLALOM checkpoint {self.waypoint_index}/{len(self.mission)-1}: next {self.goal.tolist()}')
                    else:
                        if self.landing_target is None:
                            try:site,approach=landing_site(self.world_body[:2,3],self.goal,elapsed,self.static,self.profile)
                            except ValueError as error:self.fail(str(error));return
                            self.landing_target=site
                            if np.linalg.norm(site-self.goal)>.05:
                                self.plan_revision+=1;self.goal=site;self.mission=[tuple(site)];self.waypoint_index=0
                                self.path=approach;self.path_goal_index=0;self.plan_valid=False
                                self.manual_rejoin=True;self.rejoin_at=now
                                self.event('LANDING APPROACH: moving to clear site '+str(site.tolist()));return
                        self.goal_reached=True;self.hold_since=None;self.transition('GOAL_HOLD')
                elif now-self.phase_at>150:self.fail('Navigation exceeded 150 seconds')
        elif self.phase=='GOAL_HOLD':
            target=self.landing_target
            if target is None or not landing_segment_clear(self.world_body[:2,3],target,self.static,self.profile):
                self.fail('Final landing alignment lacks checked clearance');return
            world_point=np.r_[target,self.world_body[2,3]]
            point=self.calibration[:3,:3]@world_point+self.calibration[:3,3]
            self.stream(position=np.array([point[1],point[0],self.takeoff_target[2]]))
            settled=(np.linalg.norm(self.world_body[:2,3]-target)<.12 and np.linalg.norm(self.odom.velocity)<.12)
            if settled:
                if self.hold_since is None:self.hold_since=now
                elif now-self.hold_since>1.5:self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND);self.transition('LANDING')
            else:self.hold_since=None
            if now-self.phase_at>20:self.fail('Landing alignment did not settle')

    def report(self):
        now=time.monotonic()
        preview_ready=(self.plan_at is not None and now-self.plan_at<.5 and self.plan_valid and
                       self.movers_ready and self.failure is None and self.phase=='PREVIEW')
        return {'status':('PASS: goal reached and landed/disarmed' if self.goal_reached and self.phase=='DONE' and not self.failure
                          else (('PASS: preview only; no flight commands' if preview_ready else 'NOT PASSED: preview only; no flight commands') if not self.fly else 'FLIGHT NOT COMPLETED')),
                'phase':self.phase,'failure':self.failure,'goal_reached':self.goal_reached,
                'manual_control_enabled':self.manual is not None,'manual_used':self.manual_used,'collision':self.collision,
                'landing_target_world_xy':self.landing_target.tolist() if self.landing_target is not None else None,
                'profile':self.profile,'mission_waypoints_world_xy':self.mission,
                'completed_waypoints':self.completed_waypoints,
                'world_to_px4_enu':self.calibration.tolist() if self.calibration is not None else None,
                'last_alignment':self.last_alignment,
                'telemetry_ages_s':{'odometry':now-self.odom_at if self.odom_at is not None else None,
                                    'gazebo_pose':now-self.pose_at if self.pose_at is not None else None,
                                    'vehicle_status':now-self.status_at if self.status_at is not None else None,
                                    'dds_sync':now-self.sync_at if self.sync_at is not None else None,
                                    'landed':now-self.land_at if self.land_at is not None else None},
                'observed_vehicle_pose_names':self.last_pose_names,
                'landed':self.landed,
                'armed':self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED if self.status else None,
                'command_timestamp_strategy':'Fresh PX4 DDS-adjusted feedback timestamp; OS wall-clock difference is diagnostic only',
                'goal_world_xy_m':list(self.mission[-1]),'minimum_measured_body_clearance_m':self.minimum_clearance if math.isfinite(self.minimum_clearance) else None,
                'maximum_control_timer_interval_s':self.maximum_timer_interval,
                'preview_planner_ready':self.plan_at is not None and self.plan_valid,'planner_blocked_cycles':self.plan_blocks,
                'moving_obstacles_created':self.movers_ready,'events':self.events,'command_acks':self.acknowledgements,
                'method':'PX4 offboard position takeoff and waypoint-guided velocity navigation; ground-truth A* around full movement envelopes plus predicted moving-obstacle rollouts. SANDO is not driving this separate demo.',
                'limits':'Exact simulator map/obstacle trajectories. Not sensor-based autonomy. Moving objects freeze on stop/landing/failure. Physical PX4 tracking and runtime outcome require host flight test.'}


def main():
    p=argparse.ArgumentParser();p.add_argument('--fly',action='store_true');p.add_argument('--manual-control',action='store_true');p.add_argument('--wait-for-start',action='store_true');p.add_argument('--preview-seconds',type=float,default=20);profiles=p.add_mutually_exclusive_group();profiles.add_argument('--slalom',action='store_true');profiles.add_argument('--challenge',action='store_true');a=p.parse_args()
    if a.wait_for_start and not (a.fly and a.manual_control):p.error('--wait-for-start requires --fly --manual-control')
    OUT.mkdir(exist_ok=True);rclpy.init();node=Demo(a.fly,'challenge' if a.challenge else ('slalom' if a.slalom else 'baseline'),a.manual_control,a.wait_for_start)
    try:
        node.event('PX4 FLIGHT ENABLED' if a.fly else 'PREVIEW: moving obstacles only; no PX4 control publishers')
        end=time.monotonic()+(230 if a.fly else a.preview_seconds)
        while (a.wait_for_start or time.monotonic()<end) and node.phase!='ABORT' and (a.wait_for_start or node.phase!='DONE'):
            rclpy.spin_once(node,timeout_sec=.05)
            if a.wait_for_start and node.run_started is not None and node.phase not in ('PREFLIGHT','DONE','LANDING') and time.monotonic()-node.run_started>230:node.fail('Demo overall timeout')
        if a.fly and node.phase not in ('DONE','ABORT'):
            node.fail('Demo overall timeout')
            landing_deadline=time.monotonic()+40
            while time.monotonic()<landing_deadline and node.phase not in ('DONE','ABORT'):rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:
        if a.fly and (node.owns_flight or node.arming_requested):
            node.fail('User interrupted demo; landing requested')
            deadline=time.monotonic()+40
            while time.monotonic()<deadline and node.phase not in ('DONE','ABORT'):rclpy.spin_once(node,timeout_sec=.05)
    except Exception as error:
        node.fail('Controller exception: '+str(error))
        if a.fly and (node.owns_flight or node.arming_requested):
            deadline=time.monotonic()+40
            while time.monotonic()<deadline and node.phase not in ('DONE','ABORT'):
                rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if node.manual is not None:node.manual.stop(node.failure or 'Flight ended')
        if node.command_server is not None:
            # Leave terminal outcome visible long enough for the browser to receive it.
            time.sleep(.3);node.command_server.stop()
        node.stop_thread.set();node.worker.join(timeout=6)
        if node.obstacle_rpc is not None:node.obstacle_rpc.close()
        report=node.report();(OUT/('flight_result.json' if a.fly else 'preview_result.json')).write_text(json.dumps(report,indent=2)+'\n')
        (OUT/('flight_alignment.json' if a.fly else 'preview_alignment.json')).write_text(json.dumps(node.alignment_samples)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        if node.manual is not None:node.gz.unsubscribe('/demo/contacts')
        node.gz.unsubscribe('/world/default/pose/info');node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
