import math
import time

import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import DynTraj


class TrajectoryPreview(Node):
    def __init__(self):
        super().__init__('sando_to_px4_preview')

        self.create_subscription(
            DynTraj,
            '/trajs',
            self.callback,
            10
        )

        self.last_print = 0.0
        print('Read-only SANDO trajectory preview started.')
        print('No flight commands will be sent.')

    def callback(self, msg):
        if msg.mode != 'pwp':
            return

        traj = msg.pwp
        times = list(traj.times)
        now = time.time()

        if len(times) < 2 or not all(math.isfinite(t) for t in times):
            return

        if not all(times[i] < times[i+1] for i in range(len(times)-1)):
            return

        if not (times[0] <= now < times[-1]):
            return

        segment_count = len(times) - 1

        if not all(len(c) == segment_count for c in
                   (traj.coeff_x, traj.coeff_y, traj.coeff_z)):
            return

        i = next(
            j for j in range(segment_count)
            if times[j] <= now < times[j+1]
        )

        t = now - times[i]

        def evaluate(coeff):
            position = coeff.a*t**3 + coeff.b*t**2 + coeff.c*t + coeff.d
            velocity = 3*coeff.a*t**2 + 2*coeff.b*t + coeff.c
            return position, velocity

        x, vx = evaluate(traj.coeff_x[i])
        y, vy = evaluate(traj.coeff_y[i])
        z, vz = evaluate(traj.coeff_z[i])

        values = (x, y, z, vx, vy, vz)
        if not all(math.isfinite(v) for v in values):
            return

        # ROS ENU -> PX4 NED
        px4_position = (y, x, -z)
        px4_velocity = (vy, vx, -vz)

        if now - self.last_print >= 1.0:
            self.last_print = now

            print(
                'PX4 NED position:',
                tuple(round(v, 3) for v in px4_position)
            )
            print(
                'PX4 NED velocity:',
                tuple(round(v, 3) for v in px4_velocity)
            )
            print(
                'Trajectory time remaining:',
                round(times[-1] - now, 3), 's'
            )
            print('---')


def main():
    rclpy.init()
    node = TrajectoryPreview()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
