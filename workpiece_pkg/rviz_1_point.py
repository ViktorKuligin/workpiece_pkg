#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class RvizPoint(Node):
    def __init__(self):
        super().__init__('point_node')
        self.get_logger().warn("point node start")

        self.pub = self.create_publisher(PointStamped, "my_point", 10)
        self.tim = self.create_timer(1.0, self.point_timer)

    def point_timer(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = 2.0
        msg.point.y = 4.0
        msg.point.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info(str(msg.point))

def main(args=None):
    rclpy.init(args=args)
    node = RvizPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()