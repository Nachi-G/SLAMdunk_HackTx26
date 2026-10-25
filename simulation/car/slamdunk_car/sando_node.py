"""SLAM/odometry to SANDO, and planar SANDO setpoints to guarded wheel commands.

The ROS imports are optional until the executable is started. This node neither
runs fake_sim nor invents SLAM poses, sensor measurements or map free space.
"""
import math
import time
from .control import source_stamp,track_setpoint
from .mapping import project_grid,columns,quaternion_yaw,transform_point


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid,Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32,read_points
    from std_msgs.msg import Header,String
    from dynus_interfaces.msg import State,Goal
    from tf2_ros import Buffer,TransformListener,TransformException

    class Adapter(Node):
        def __init__(self):
            super().__init__('car_sando_adapter')
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            self.pose=None;self.pose_at=self.map_at=-math.inf;self.stamps={};self.hits=set();self.hits_at=-math.inf;self.last_clock=None;self.grid_message=None;self.cloud_stamp=None
            self.state_pub=self.create_publisher(State,'/car_sando/state',1)
            self.pose_pub=self.create_publisher(Odometry,'/car/map_odom',1)
            self.occupied_pub=self.create_publisher(PointCloud2,'/car_sando/occupancy_grid',qos_profile_sensor_data)
            self.unknown_pub=self.create_publisher(PointCloud2,'/car_sando/unknown_grid',qos_profile_sensor_data)
            self.command_pub=self.create_publisher(Twist,'/car/cmd_vel_sando',1)
            self.status=self.create_publisher(String,'/car/sando_status',1)
            self.create_subscription(Odometry,'/car/odom',self.odom,qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid,'/map',self.grid,1)
            self.create_subscription(PointCloud2,'/car/points',self.cloud,qos_profile_sensor_data)
            self.create_subscription(Goal,'/car_sando/goal',self.goal,1)
            self.last_warning=0.
            self.create_timer(.5,self.publish_map)

        def warn(self,reason):
            msg=String();msg.data=reason;self.status.publish(msg)
            if time.monotonic()-self.last_warning>2:self.get_logger().warning(reason);self.last_warning=time.monotonic()

        def valid(self,message,kind,max_age=.5):
            now=self.get_clock().now().nanoseconds/1e9
            if self.last_clock is not None and now<self.last_clock:
                self.pose=None;self.pose_at=self.map_at=-math.inf;self.stamps.clear();self.hits.clear();self.grid_message=None;self.cloud_stamp=None
            self.last_clock=now
            stamp=source_stamp(message.header.stamp.sec,message.header.stamp.nanosec,now,max_age)
            if stamp<=self.stamps.get(kind,-1):raise ValueError('out of order '+kind)
            self.stamps[kind]=stamp
            return Time.from_msg(message.header.stamp)

        def odom(self,msg):
            try:
                at=self.valid(msg,'odom')
                if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':raise ValueError('invalid odometry frames')
                # Use acquisition-time SLAM TF, never latest TF or simulator truth.
                t=self.tf.lookup_transform('map','base_link',at)
                p=t.transform.translation;q=t.transform.rotation
                yaw=quaternion_yaw((q.x,q.y,q.z,q.w))
                self.pose=(p.x,p.y,yaw);self.pose_at=time.monotonic()
                v=msg.twist.twist.linear;vx=math.cos(yaw)*v.x-math.sin(yaw)*v.y;vy=math.sin(yaw)*v.x+math.cos(yaw)*v.y
                if not all(math.isfinite(n) for n in (*self.pose,vx,vy)):raise ValueError('nonfinite odometry')
                state=State();state.header.stamp=msg.header.stamp;state.header.frame_id='map'
                state.pos.x=float(p.x);state.pos.y=float(p.y);state.pos.z=.3
                state.vel.x=float(vx);state.vel.y=float(vy);state.vel.z=0.;state.quat=q
                self.state_pub.publish(state)
                out=Odometry();out.header=state.header;out.child_frame_id='base_link';out.pose.pose.position.x=float(p.x);out.pose.pose.position.y=float(p.y);out.pose.pose.position.z=float(p.z);out.pose.pose.orientation=q;out.twist=msg.twist
                self.pose_pub.publish(out)
            except (ValueError,TransformException) as error:self.warn('Waiting for SLAM pose: '+str(error))

        def cloud(self,msg):
            try:
                at=self.valid(msg,'cloud')
                if msg.header.frame_id!='lidar3d' or msg.width*msg.height>200000 or len(msg.data)>8000000:raise ValueError('invalid cloud')
                t=self.tf.lookup_transform('map','lidar3d',at).transform
                translation=(t.translation.x,t.translation.y,t.translation.z);rotation=(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
                hits=set()
                for p in read_points(msg,field_names=('x','y','z'),skip_nans=True):
                    point=tuple(float(n) for n in p)
                    if not all(math.isfinite(n) for n in point):continue
                    # Remove floor, overhead returns and the vehicle's own body.
                    if not .04<=point[2]+.6<=.68 or (abs(point[0])<.30 and abs(point[1])<.24):continue
                    x,y,z=transform_point(point,translation,rotation)
                    if .04<=z<=.68 and -12<=x<12 and -9<=y<9:hits.add((math.floor((x+12)/.2),math.floor((y+9)/.2)))
                self.hits=hits;self.hits_at=time.monotonic();self.cloud_stamp=msg.header.stamp
            except (ValueError,TransformException,AssertionError,TypeError):pass

        def grid(self,msg):
            try:
                # Occupancy grids are persistent snapshots. A stationary SLAM
                # graph may retain its last scan stamp; freshness comes from
                # current LiDAR and acquisition-time pose, not a fake map age.
                now=self.get_clock().now().nanoseconds/1e9
                stamp=source_stamp(msg.header.stamp.sec,msg.header.stamp.nanosec,now,86400.)
                if stamp<self.stamps.get('map',-1):raise ValueError('out of order map')
                if msg.header.frame_id!='map':raise ValueError('SLAM grid must be in map')
                q=msg.info.origin.orientation
                if abs(q.x)+abs(q.y)+abs(q.z)>.0001 or abs(abs(q.w)-1)>.0001:raise ValueError('rotated occupancy origins are unsupported')
                if msg.info.width*msg.info.height>2_000_000:raise ValueError('map too large')
                self.grid_message=msg;self.stamps['map']=stamp
            except (ValueError,TypeError) as error:self.warn('Map rejected: '+str(error))

        def publish_map(self):
            try:
                msg=self.grid_message
                if msg is None or self.cloud_stamp is None or time.monotonic()-self.hits_at>.35 or time.monotonic()-self.pose_at>.35:return
                origin=msg.info.origin.position
                occupied,unknown,_,_,_=project_grid(list(msg.data),msg.info.width,msg.info.height,msg.info.resolution,(origin.x,origin.y),hits=self.hits)
                # Snapshot columns plus this accepted raw observation. Preserve
                # that observation's real source stamp; never stamp stale data now.
                header=Header();header.frame_id='map';header.stamp=self.cloud_stamp
                self.occupied_pub.publish(create_cloud_xyz32(header,columns(occupied)))
                self.unknown_pub.publish(create_cloud_xyz32(header,columns(unknown)))
                self.map_at=time.monotonic()
            except (ValueError,TypeError) as error:self.warn('Map rejected: '+str(error))

        def goal(self,msg):
            command=Twist()
            try:
                self.valid(msg,'goal')
                if msg.header.frame_id!='map':raise ValueError('setpoint frame must be map')
                if self.pose is None or time.monotonic()-self.pose_at>.35 or time.monotonic()-self.map_at>1.5:raise ValueError('stale SLAM pose/map')
                if abs(msg.a.z)>.02 or abs(msg.j.z)>.02:raise ValueError('nonplanar acceleration/jerk')
                v,w=track_setpoint(self.pose,(msg.p.x,msg.p.y,msg.p.z),(msg.v.x,msg.v.y,msg.v.z),msg.yaw,msg.dyaw)
                command.linear.x=float(v);command.angular.z=float(w)
            except (ValueError,TypeError) as error:self.warn('Setpoint rejected: '+str(error))
            self.command_pub.publish(command)

    rclpy.init(args=args);node=Adapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
