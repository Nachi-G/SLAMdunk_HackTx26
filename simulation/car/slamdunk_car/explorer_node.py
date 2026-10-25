"""Opt-in 2D car explorer. It publishes destinations only; SANDO plans motion."""
import math
import time
from .control import source_stamp
from .mapping import project_grid
from .exploration import select_frontier


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid,Odometry
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String,Bool

    class Explorer(Node):
        def __init__(self):
            super().__init__('car_frontier_explorer')
            self.pose=None;self.pose_at=-math.inf;self.map_at=-math.inf;self.map_stamp=-1.;self.map_sequence=0
            self.scan_at=-math.inf;self.scan_stamp=-1.;self.grid_data=None;self.last_cells=None;self.excluded=set();self.active=None;self.started=0.;self.best=math.inf;self.progress=0.;self.arrived_at=None;self.confirmations=0;self.confirmed_sequence=-1
            self.pub=self.create_publisher(PoseStamped,'/car_sando/term_goal',1)
            self.status=self.create_publisher(String,'/car/exploration_status',1)
            self.gate=self.create_publisher(Bool,'/car/exploration_active',1)
            self.create_subscription(Odometry,'/car/map_odom',self.odom,1)
            self.create_subscription(OccupancyGrid,'/map',self.grid,1)
            self.create_subscription(LaserScan,'/car/scan',self.scan,qos_profile_sensor_data)
            self.create_timer(1.,self.tick)

        def report(self,text,active=False):
            msg=String();msg.data=text;self.status.publish(msg)
            gate=Bool();gate.data=active;self.gate.publish(gate)

        def odom(self,msg):
            try:
                source_stamp(msg.header.stamp.sec,msg.header.stamp.nanosec,self.get_clock().now().nanoseconds/1e9)
                p=msg.pose.pose.position
                if msg.header.frame_id!='map' or not all(math.isfinite(v) for v in (p.x,p.y)):return
                self.pose=(p.x,p.y);self.pose_at=time.monotonic()
            except ValueError:pass

        def scan(self,msg):
            try:
                stamp=source_stamp(msg.header.stamp.sec,msg.header.stamp.nanosec,self.get_clock().now().nanoseconds/1e9)
                if msg.header.frame_id!='laser' or stamp<=self.scan_stamp or len(msg.ranges)<180:return
                if any(math.isnan(r) or r<msg.range_min for r in msg.ranges):return
                self.scan_stamp=stamp;self.scan_at=time.monotonic();self.map_sequence+=1
            except ValueError:pass

        def grid(self,msg):
            try:
                stamp=source_stamp(msg.header.stamp.sec,msg.header.stamp.nanosec,self.get_clock().now().nanoseconds/1e9,86400.)
                q=msg.info.origin.orientation
                if msg.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>.0001:return
                if stamp<self.map_stamp:return
                _,_,cells,nx,ny=project_grid(list(msg.data),msg.info.width,msg.info.height,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))
                if self.last_cells is not None and cells!=self.last_cells:self.excluded.clear();self.confirmations=0
                self.grid_data=(cells,nx,ny);self.last_cells=cells;self.map_at=time.monotonic();self.map_stamp=stamp
            except ValueError:pass

        def tick(self):
            now=time.monotonic()
            if self.pose is None or self.grid_data is None or now-self.pose_at>.5 or now-self.scan_at>.35:
                self.report('waiting: SLAM map and fresh pose/scan required');return
            # Prevent two destination writers from racing. No goal_sender or
            # second supervisor may publish on this dedicated car goal topic.
            if len(self.get_publishers_info_by_topic('/car_sando/term_goal'))!=1:
                self.report('blocked: another destination publisher is present');return
            cells,nx,ny=self.grid_data
            if self.active is not None:
                gx=-12+(self.active%nx+.5)*.2;gy=-9+(self.active//nx+.5)*.2
                d=math.hypot(self.pose[0]-gx,self.pose[1]-gy)
                if d<self.best-.05:self.best=d;self.progress=now
                if d<.3:
                    if self.arrived_at is None:self.arrived_at=(now,self.map_sequence)
                    if now-self.arrived_at[0]<1.5 or self.map_sequence<=self.arrived_at[1]:self.report('inspecting viewpoint');return
                elif now-self.progress<15.:self.report('exploring: travelling to viewpoint',True);return
                self.excluded.add(self.active);self.active=None;self.arrived_at=None
            ix=math.floor((self.pose[0]+12)/.2);iy=math.floor((self.pose[1]+9)/.2)
            if not (0<=ix<nx and 0<=iy<ny):self.report('blocked: car outside mission bounds');return
            result=select_frontier(cells,nx,ny,iy*nx+ix,excluded=self.excluded)
            if result['state']=='complete':
                if self.map_sequence!=self.confirmed_sequence:self.confirmations+=1;self.confirmed_sequence=self.map_sequence
                if self.confirmations<4:self.report('confirming frontier exhaustion');return
            else:self.confirmations=0
            self.report(result['state']+': '+result['reason'],result['state']=='exploring')
            if result['goal'] is None:return
            self.active=result['goal'];self.best=math.inf;self.progress=now
            msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x=float(-12+(self.active%nx+.5)*.2);msg.pose.position.y=float(-9+(self.active//nx+.5)*.2);msg.pose.position.z=.3;msg.pose.orientation.w=1.
            self.pub.publish(msg)

    rclpy.init(args=args);node=Explorer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
