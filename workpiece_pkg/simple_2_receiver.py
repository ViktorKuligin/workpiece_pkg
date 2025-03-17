#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

class SimpleRx(Node):
    def __init__(self):
        super().__init__('rx_node')
        self.get_logger().warn("rx node start")

        self.sub = self.create_subscription(String, "msg", self.string_cb, 10)

    def string_cb(self, msg: String):
        msg_in = msg.data
        self.get_logger().info(msg_in)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRx()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()