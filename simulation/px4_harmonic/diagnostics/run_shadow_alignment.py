"""Shadow planner fed by existing telemetry/map; never publishes to PX4 inputs."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import DynTraj
from geometry_msgs.msg import PoseStamped
import yaml

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics/shadow_alignment'
OUT.mkdir(exist_ok=True)
(OUT/'summary.json').write_text(json.dumps({'status':'RUNNING: no result yet'}))


def stop(child):
    if child is None:return
    try:os.killpg(child.pid,signal.SIGINT)
    except ProcessLookupError:pass
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:os.killpg(child.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        child.wait()


class GoalMirror(Node):
    def __init__(self):
        super().__init__('sando_shadow_goal_bootstrap')
        self.goal=None
        self.sent=False
        self.shadow_count=0
        self.publisher=self.create_publisher(PoseStamped,'/SH01/term_goal',10)
        self.create_subscription(DynTraj,'/trajs',self.source,10)
        self.create_subscription(DynTraj,'/sando_shadow/trajs',self.shadow,10)

    def source(self,msg):
        if msg.id==1 and msg.is_agent and len(msg.goal)==3 and all(math.isfinite(v) for v in msg.goal):
            self.goal=list(msg.goal)

    def shadow(self,msg):
        if msg.id==1 and msg.is_agent:self.shadow_count+=1

    def send(self):
        if self.sent or self.goal is None or self.publisher.get_subscription_count()!=1:return
        goal=PoseStamped()
        goal.header.frame_id='map'
        goal.header.stamp=self.get_clock().now().to_msg()
        goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=self.goal
        goal.pose.orientation.w=1.
        self.publisher.publish(goal)
        self.sent=True


def main():
    # Preserve the original YAML; generate a separate namespace-specific copy.
    original=yaml.safe_load((ROOT/'sando_harmonic/config/sando_nx01.yaml').read_text())
    params=dict(original['/NX01/sando_node']['ros__parameters'])
    params.update(sim_env='fake_sim',use_global_pc=False,use_state_update=False,
                  use_dynamic_factor=False,factor_initial=3.,factor_final=4.,
                  factor_constant_step_size=.25,use_benchmark=False,visual_level=0,
                  debug_verbose=True,ignore_other_trajs=True)
    config=OUT/'shadow_parameters.yaml'
    config.write_text(yaml.safe_dump({'/SH01/sando_node':{'ros__parameters':params}}))
    env=dict(os.environ,ROS_LOG_DIR=str(OUT/'ros_logs'))
    command=[str(ROOT/'sando_harmonic_ws/install/sando/lib/sando/sando'),'--ros-args',
             '-r','__ns:=/SH01','--params-file',str(config),
             '-r','state:=/NX01/state','-r','sensor_point_cloud:=/NX01/sensor_point_cloud',
             '-r','/trajs:=/sando_shadow/trajs']
    shadow=observer=None
    rclpy.init()
    node=GoalMirror()
    # Never publish goals to an already existing shadow planner.
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if node.publisher.get_subscription_count():
        node.destroy_node();rclpy.shutdown()
        raise RuntimeError('Existing /SH01 goal subscriber found; refusing to start overlapping shadow test')
    try:
        with (OUT/'planner.log').open('w') as log, (OUT/'preview.log').open('w') as preview_log:
            shadow=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            print('Shadow planning only: reads existing map/state; all trajectory outputs remapped.',flush=True)
            deadline=time.monotonic()+15
            while not node.sent and time.monotonic()<deadline:
                if shadow.poll() is not None:raise RuntimeError('Shadow planner exited; inspect planner.log')
                rclpy.spin_once(node,timeout_sec=.1)
                node.send()
            if not node.sent:raise RuntimeError('Could not capture current goal or discover shadow planner')
            observer=subprocess.Popen(['python3',str(ROOT/'sando_to_px4_dry_run.py'),
                '--trajectory-topic','/sando_shadow/trajs','--seconds','15',
                '--output',str(OUT/'preview.json')],env=env,stdout=preview_log,
                stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+22
            while observer.poll() is None:
                if shadow.poll() is not None:raise RuntimeError('Shadow planner exited during observation')
                if time.monotonic()>deadline:raise RuntimeError('Read-only observer exceeded deadline')
                rclpy.spin_once(node,timeout_sec=.1)
            if observer.returncode:raise RuntimeError('Observer failed; inspect preview.log')
            preview=json.loads((OUT/'preview.json').read_text())
            samples=preview.get('alignment_samples',[])
            passed=(preview['valid_20hz_evaluations']>=100 and preview['validated_trajectories']>=5
                    and not preview['rejected_trajectories'] and preview['maximum_timer_interval_s']<.2)
            summary={'status':'PASS: live-input shadow trajectory timing/preview' if passed else 'FAIL: shadow timing/preview needs investigation',
                     'goal_copied_from_current_planner':node.goal,'shadow_trajectories_observed':node.shadow_count,
                     'validated_trajectories':preview['validated_trajectories'],
                     'valid_20hz_evaluations':preview['valid_20hz_evaluations'],
                     'maximum_start_age_s':max((s['trajectory_start_age_s'] for s in samples),default=None),
                     'maximum_reference_position_difference_m':max((s['position_difference_m'] for s in samples),default=None),
                     'rejected_trajectories':preview['rejected_trajectories'],'watchdog_blocks':preview['watchdog_blocks'],
                     'limits':'Uses current map source, which is still synthetic. No drone motion, onboard LiDAR, flight publishing, or obstacle tracking demonstrated.'}
            (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            print(json.dumps(summary,indent=2),flush=True)
            return 0 if passed else 1
    except (RuntimeError,OSError,ValueError) as error:
        (OUT/'summary.json').write_text(json.dumps({'status':f'FAIL: {error}'}))
        print(error,flush=True)
        return 1
    finally:
        stop(observer);stop(shadow)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
