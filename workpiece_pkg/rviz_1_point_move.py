#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class RvizPointMove(Node):
    def __init__(self):
        super().__init__('point_move_node')
        self.get_logger().warn("point move node start")

        self.radius = 3.0
        self.degree = 0.0

        self.pub = self.create_publisher(PointStamped, "my_point_move", 10)
        self.tim = self.create_timer(0.01, self.point_move_timer)

    def point_move_timer(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = np.cos(self.degree * np.pi / 180) * self.radius
        msg.point.y = np.sin(self.degree * np.pi / 180) * self.radius
        msg.point.z = 0.0

        self.pub.publish(msg)
        self.get_logger().info(str(self.degree) + ' ' + str(msg.point))

        self.degree += 1
        if self.degree == 360:
            self.degree = 0

def main(args=None):
    rclpy.init(args=args)
    node = RvizPointMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()