import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math

rclpy.init()
node = Node('test_tree_publisher')

pub = node.create_publisher(
    PointCloud2, '/NX01/sensor_point_cloud', 10
)

# Tree trunk centered at (3, 0)
# Diameter = 0.6 m, height = 3 m
points = []

for z_step in range(16):
    z = z_step * 0.2
    for angle_step in range(24):
        angle = angle_step * 2 * math.pi / 24
        x = 3.0 + 0.3 * math.cos(angle)
        y = 0.3 * math.sin(angle)
        points.append((x, y, z))

msg = PointCloud2()
msg.header.frame_id = 'map'
msg.height = 1
msg.width = len(points)
msg.fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
msg.is_bigendian = False
msg.point_step = 12
msg.row_step = msg.point_step * msg.width
msg.is_dense = True
msg.data = b''.join(struct.pack('<fff', *p) for p in points)

print(f'Publishing tree trunk with {len(points)} points', flush=True)

while rclpy.ok():
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
