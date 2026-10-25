import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from dynus_interfaces.msg import State


def quaternion_multiply(a, b):
    # Quaternions are represented as (w, x, y, z).
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


class PX4ToSANDO(Node):
    def __init__(self):
        super().__init__('px4_to_sando_bridge')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(State, '/NX01/state', 10)

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos,
        )

        self.count = 0
        self.get_logger().info('PX4-to-SANDO telemetry bridge started')

    def odometry_callback(self, odom):
        # Only accept NED position and NED velocity.
        if odom.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return

        if odom.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            return

        values = list(odom.position) + list(odom.velocity) + list(odom.q)
        if not all(math.isfinite(v) for v in values):
            return

        state = State()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'

        # NED -> ENU
        state.pos.x = float(odom.position[1])
        state.pos.y = float(odom.position[0])
        state.pos.z = -float(odom.position[2])

        state.vel.x = float(odom.velocity[1])
        state.vel.y = float(odom.velocity[0])
        state.vel.z = -float(odom.velocity[2])

        # PX4: NED world, FRD body, quaternion (w,x,y,z).
        # ROS: ENU world, FLU body, quaternion fields (x,y,z,w).
        q_ned_frd = tuple(float(v) for v in odom.q)
        q_enu_ned = (0.0, 2**-0.5, 2**-0.5, 0.0)
        q_frd_flu = (0.0, 1.0, 0.0, 0.0)

        q = quaternion_multiply(
            quaternion_multiply(q_enu_ned, q_ned_frd),
            q_frd_flu,
        )

        norm = math.sqrt(sum(v*v for v in q))
        if norm < 1e-6:
            return

        state.quat.w = q[0] / norm
        state.quat.x = q[1] / norm
        state.quat.y = q[2] / norm
        state.quat.z = q[3] / norm

        self.publisher.publish(state)

        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(
                f'Publishing PX4 state to SANDO: '
                f'x={state.pos.x:.2f}, '
                f'y={state.pos.y:.2f}, '
                f'z={state.pos.z:.2f}'
            )


rclpy.init()
node = PX4ToSANDO()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
