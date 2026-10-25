"""Single command publisher with wall-time watchdog, sensor stop and stop latch."""
import math
import threading
import time
from .control import DriveGuard,source_stamp


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points
    from std_msgs.msg import String,Bool
    from std_srvs.srv import SetBool

    class GuardNode(Node):
        def __init__(self):
            super().__init__('car_drive_guard')
            self.declare_parameter('mode','teleop');mode=self.get_parameter('mode').value
            if mode not in ('teleop','sando'):raise ValueError('mode must be teleop or sando')
            self.declare_parameter('require_exploration_gate',False)
            self.require_gate=self.get_parameter('require_exploration_gate').value;self.gate=False;self.gate_at=-math.inf
            self.create_subscription(Bool,'/car/exploration_active',self.exploration_gate,1)
            self.guard=DriveGuard();self.lock=threading.RLock();self.last_clock=None
            self.stamps={};self.done=threading.Event()
            self.pub=self.create_publisher(Twist,'/car/cmd_vel',1)
            self.status=self.create_publisher(String,'/car/drive_status',1)
            self.create_subscription(Twist,'/car/cmd_vel_'+mode,self.request,1)
            self.create_subscription(LaserScan,'/car/scan',self.scan,qos_profile_sensor_data)
            self.create_subscription(PointCloud2,'/car/points',self.cloud,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/car/odom',self.odom,qos_profile_sensor_data)
            self.create_service(SetBool,'/car/stop',self.stop)
            self.worker=threading.Thread(target=self.watch,daemon=True);self.worker.start()

        def valid(self,message,kind,frame):
            if message.header.frame_id!=frame:raise ValueError('unexpected '+kind+' frame')
            now=self.get_clock().now().nanoseconds/1e9
            if self.last_clock is not None and now<self.last_clock:
                self.guard.reset();self.stamps.clear()
            self.last_clock=now
            stamp=source_stamp(message.header.stamp.sec,message.header.stamp.nanosec,now)
            if stamp<=self.stamps.get(kind,-1):raise ValueError('out of order '+kind)
            self.stamps[kind]=stamp

        def exploration_gate(self,msg):
            with self.lock:self.gate=msg.data;self.gate_at=time.monotonic()

        def request(self,msg):
            with self.lock:
                if any(not math.isfinite(v) or v!=0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                    self.guard.request_at=-math.inf;return
                self.guard.set_request(msg.linear.x,msg.angular.z,time.monotonic())

        def scan(self,msg):
            with self.lock:
                try:
                    self.valid(msg,'scan','laser')
                    if len(msg.ranges)<180 or abs(msg.angle_max-msg.angle_min)<6.0:return
                    self.guard.scan(msg.ranges,msg.range_min,msg.range_max,time.monotonic())
                except (ValueError,TypeError):pass

        def cloud(self,msg):
            with self.lock:
                try:
                    self.valid(msg,'cloud','lidar3d')
                    if msg.width*msg.height>200000 or len(msg.data)>8000000:return
                    points=read_points(msg,field_names=('x','y','z'),skip_nans=True)
                    self.guard.cloud(((float(p[0]),float(p[1]),float(p[2])+.60) for p in points),time.monotonic())
                except (ValueError,TypeError,AssertionError,IndexError):pass

        def odom(self,msg):
            with self.lock:
                try:
                    self.valid(msg,'odom','odom')
                    if msg.child_frame_id!='base_link':return
                    v=msg.twist.twist.linear
                    self.guard.odom(math.hypot(v.x,v.y),time.monotonic())
                except (ValueError,TypeError):pass

        def stop(self,request,response):
            with self.lock:
                self.guard.estopped=request.data
                # Clearing the stop latch never revives the old request.
                self.guard.request_at=-math.inf
            response.success=True;response.message='Stopped' if request.data else 'Stop cleared; send a new command'
            return response

        def publish(self,command):
            msg=Twist();msg.linear.x=float(command[0]);msg.angular.z=float(command[1]);self.pub.publish(msg)

        def watch(self):
            previous=time.monotonic();last_status=0.
            while not self.done.wait(.025):
                now=time.monotonic()
                with self.lock:
                    if self.require_gate and (not self.gate or now-self.gate_at>1.5):
                        self.guard.command=(0.,0.);command=(0.,0.);reason='exploration not active'
                    else:command=self.guard.step(now,min(now-previous,.1));reason=self.guard.reason
                self.publish(command);previous=now
                if now-last_status>.5:
                    msg=String();msg.data=reason;self.status.publish(msg);last_status=now

        def close(self):
            self.done.set();self.worker.join(timeout=1);self.publish((0.,0.))

    rclpy.init(args=args);node=GuardNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.close()
        else:node.done.set();node.worker.join(timeout=1)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
