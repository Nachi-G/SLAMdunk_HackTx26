"""Observe PX4's GCS heartbeat state; never publishes messages."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleStatus


def main():
    rclpy.init();node=Node('read_only_qgc_heartbeat_check');samples=[]
    def receive(msg):samples.append({'received_mono':time.monotonic(),'gcs_connection_lost':msg.gcs_connection_lost,
                                     'armed':msg.arming_state==VehicleStatus.ARMING_STATE_ARMED})
    node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',receive,qos_profile_sensor_data)
    try:
        end=time.monotonic()+8
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        good=bool(samples) and time.monotonic()-samples[-1]['received_mono']<1 and all(not s['gcs_connection_lost'] for s in samples[-3:])
        result={'status':'PASS: PX4 receives GCS heartbeats' if good else 'NOT CONFIRMED: ground-station heartbeat connection',
                'samples':len(samples),'last_status':samples[-1] if samples else None}
        path=Path(__file__).resolve().parent/'qgc/heartbeat.json';path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
