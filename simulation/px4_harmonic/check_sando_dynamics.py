import math
import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import DynTraj

MAX_SPEED = 1.0
MAX_ACCEL = 1.0
MAX_JERK = 2.0


class DynamicsChecker(Node):
    def __init__(self):
        super().__init__('sando_dynamics_checker')
        self.create_subscription(DynTraj, '/trajs', self.check, 10)

    def check(self, msg):
        if msg.mode != 'pwp':
            return

        pwp = msg.pwp
        times = list(pwp.times)
        count = len(times) - 1

        if count < 1:
            return

        if not all(times[i] < times[i + 1] for i in range(count)):
            print('WARNING: Invalid trajectory timestamps')
            return

        if any(len(coeffs) != count for coeffs in
               (pwp.coeff_x, pwp.coeff_y, pwp.coeff_z)):
            print('WARNING: Invalid coefficient count')
            return

        max_speed = 0.0
        max_accel = 0.0
        max_jerk = 0.0

        for i in range(count):
            duration = times[i + 1] - times[i]

            for step in range(101):
                t = duration * step / 100.0

                velocities = []
                accelerations = []
                jerks = []

                for coeffs in (pwp.coeff_x, pwp.coeff_y, pwp.coeff_z):
                    c = coeffs[i]

                    velocities.append(
                        3*c.a*t*t + 2*c.b*t + c.c
                    )
                    accelerations.append(
                        6*c.a*t + 2*c.b
                    )
                    jerks.append(6*c.a)

                speed = math.sqrt(sum(v*v for v in velocities))
                accel = math.sqrt(sum(a*a for a in accelerations))
                jerk = math.sqrt(sum(j*j for j in jerks))

                max_speed = max(max_speed, speed)
                max_accel = max(max_accel, accel)
                max_jerk = max(max_jerk, jerk)

        print('Maximum speed:', round(max_speed, 3), 'm/s')
        print('Maximum acceleration:', round(max_accel, 3), 'm/s²')
        print('Maximum jerk:', round(max_jerk, 3), 'm/s³')

        tolerance = 0.01

        if (max_speed <= MAX_SPEED + tolerance and
                max_accel <= MAX_ACCEL + tolerance and
                max_jerk <= MAX_JERK + tolerance):
            print('PASS: Sampled dynamics within configured limits')
        else:
            print('WARNING: At least one dynamics limit exceeded')

        print('---')


rclpy.init()
node = DynamicsChecker()
rclpy.spin_once(node, timeout_sec=5.0)
node.destroy_node()
rclpy.shutdown()
