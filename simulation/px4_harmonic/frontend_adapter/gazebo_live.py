"""Read-only Gazebo ground-truth stream for the cloned dashboard. No control I/O."""
import math
import json
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT.parents[1]/'integration/slam_dunk_bridge'))
from slam_dunk_bridge.core import LatestStore,MapPublisher,health_message
from frontend_adapter.control_proxy import DashboardServer
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V


def cylinder_points(x,y,z,radius,height):
    return [[x+radius*math.cos(i*math.tau/20),y+radius*math.sin(i*math.tau/20),z-height/2+j*height/10]
            for i in range(20) for j in range(11)]


def forest_points():
    points=[]
    for model in ET.parse(ROOT/'gazebo_worlds/hacktx_forest.sdf').findall('world/model'):
        cylinder=model.find('link/collision/geometry/cylinder')
        if cylinder is None:continue
        x,y,z,*_=map(float,model.findtext('pose').split())
        points+=cylinder_points(x,y,z,float(cylinder.findtext('radius')),float(cylinder.findtext('length')))
    return points


class LiveScene:
    def __init__(self):
        self.lock=threading.Lock();self.latest=None;self.received=None
    def receive(self,message):
        # Keep only the newest bounded scene snapshot; don't serialize in callback.
        by_name={p.name:p for p in message.pose if p.name=='x500_0' or p.name.startswith('quick_demo_crossing_')}
        stamp=message.header.stamp.sec*1000000000+message.header.stamp.nsec
        if stamp<=0:return
        with self.lock:self.latest=(stamp,by_name);self.received=time.monotonic()
    def snapshot(self):
        with self.lock:return self.latest,self.received


def main():
    store=LatestStore();maps=MapPublisher(store,frame_id='gazebo_world',max_points=20000)
    scene=LiveScene();node=Node();topic='/world/default/pose/info'
    if not node.subscribe(Pose_V,topic,scene.receive):raise RuntimeError('Gazebo pose subscription failed')
    server=DashboardServer(store);static=forest_points();last_stamp=0;last_map=0
    last_metrics=None
    accepted={'map':None,'pose':None};counters={'clock_rewinds':0}
    try:
        server.start()
        print('LIVE GAZEBO GROUND TRUTH — read only; not a LiDAR map.\nOpen http://localhost:8765/?source=live\nWaiting for '+topic,flush=True)
        while True:
            snapshot,received=scene.snapshot();now=time.monotonic()
            live_plan=None
            try:
                candidate=json.loads((ROOT/'diagnostics/quick_px4_demo/live_plan.json').read_text())
                age=now-candidate['updated_monotonic_s']
                if candidate.get('frame_id')=='gazebo_world' and 0<=age<1:live_plan=candidate;last_metrics=candidate
            except (OSError,ValueError,TypeError,KeyError):pass
            if snapshot and received is not None and now-received<.5:
                stamp_ns,poses=snapshot
                if stamp_ns<last_stamp:raise RuntimeError('Gazebo time reset: restart dashboard bridge')
                if stamp_ns>last_stamp:
                    stamp={'sec':stamp_ns//1000000000,'nanosec':stamp_ns%1000000000}
                    drone=poses.get('x500_0')
                    if drone is not None:
                        p=drone.position;q=drone.orientation
                        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                        if all(math.isfinite(v) for v in values) and sum(v*v for v in values[3:])>1e-9:
                            store.publish({'type':'pose','frame_id':'gazebo_world','child_frame_id':'x500_0',
                                           'stamp':stamp,'source':'gazebo_ground_truth/model_origin',
                                           'position':values[:3],'orientation':values[3:],
                                           **({'velocity':live_plan['velocity_world']} if live_plan and live_plan.get('velocity_world') is not None else {}),
                                           'battery':live_plan['battery'] if live_plan else 100.,
                                           'battery_source':live_plan['battery_source'] if live_plan else 'placeholder',
                                           'flight_seconds':last_metrics['flight_seconds'] if last_metrics else 0.,
                                           'flight_source':('controller' if live_plan else 'controller_last') if last_metrics else 'placeholder'})
                            accepted['pose']=received
                    if now-last_map>=.2:
                        points=list(static)
                        for name,pose in poses.items():
                            if name.startswith('quick_demo_crossing_'):
                                p=pose.position
                                if all(math.isfinite(v) for v in (p.x,p.y,p.z)):
                                    points+=cylinder_points(p.x,p.y,p.z,.65,3.)
                        maps.replace(points,stamp,'gazebo_ground_truth/forest_sdf_and_live_obstacles')
                        accepted['map']=received;last_map=now
                    last_stamp=stamp_ns
            status=health_message(accepted,counters,'gazebo_world')
            status['source']='gazebo_ground_truth'
            status['plan']=live_plan
            try:
                mission=json.loads((ROOT/'diagnostics/quick_px4_demo/manual_state.json').read_text())
                status['vehicle_event']=('COLLISION DETECTED — MISSION FAILED' if mission.get('collision') else mission.get('event',''))[:512]
            except (OSError,ValueError,TypeError):pass
            store.publish(status)
            if server.error:raise RuntimeError('Dashboard server failed') from server.error
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:node.unsubscribe(topic);server.stop()

if __name__=='__main__':main()
