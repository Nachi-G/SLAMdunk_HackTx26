"""Terminal keys expire unless repeated. S=stop, Q=quit; no motion at startup."""
import select
import sys
import termios
import time
import tty


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    if not sys.stdin.isatty():raise RuntimeError('Run car_teleop in an interactive terminal')
    rclpy.init(args=args);node=rclpy.create_node('car_keyboard');pub=node.create_publisher(Twist,'/car/cmd_vel_teleop',1)
    original=termios.tcgetattr(sys.stdin);last=-float('inf');v=w=0.
    print('Hold/repeat W forward, X reverse, A left, D right. S/Space stop. Q quit. First key expires after 0.25 s.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],.03)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('q','\x03'):break
                v,w={'w':(.25,0.),'x':(-.20,0.),'a':(0.,.6),'d':(0.,-.6)}.get(key,(0.,0.));last=time.monotonic()
            if time.monotonic()-last>.25:v=w=0.
            msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg);rclpy.spin_once(node,timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,original)
        if rclpy.ok():
            for _ in range(3):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
