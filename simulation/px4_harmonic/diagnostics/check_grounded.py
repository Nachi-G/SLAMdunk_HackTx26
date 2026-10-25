"""Read-only confirmation of fresh PX4 landed+disarmed status for 3 seconds."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleStatus,VehicleLandDetected


class GroundCheck(Node):
    def __init__(self):
        super().__init__('px4_read_only_ground_check')
        self.status=self.landed=None
        self.status_at=self.landed_at=None
        self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',self.vehicle,qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.land,qos_profile_sensor_data)

    def vehicle(self,msg):
        self.status=msg.arming_state
        self.status_at=time.monotonic()

    def land(self,msg):
        self.landed=msg.landed
        self.landed_at=time.monotonic()


def main():
    rclpy.init();node=GroundCheck()
    start=time.monotonic();stable=None;passed=False
    try:
        while time.monotonic()-start<40:
            rclpy.spin_once(node,timeout_sec=.1)
            now=time.monotonic()
            fresh=(node.status_at is not None and now-node.status_at<.75 and
                   node.landed_at is not None and now-node.landed_at<1.5)
            grounded=fresh and node.status==VehicleStatus.ARMING_STATE_DISARMED and node.landed is True
            if grounded:
                if stable is None:stable=now
                if now-stable>=3:passed=True;break
            else:stable=None
        report={'status':'PASS: landed and disarmed with fresh telemetry for 3 seconds' if passed
                        else 'NOT CONFIRMED: do not stop or replace the simulator',
                'arming_state':node.status,'landed':node.landed,
                'observation_wall_s':time.monotonic()-start,
                'control_commands_sent':False}
        target=Path(__file__).resolve().parent/'grounded_status.json'
        target.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        return 0 if passed else 1
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
