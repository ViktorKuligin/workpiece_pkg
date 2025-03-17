#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

class SimpleTx(Node):
    def __init__(self):
        super().__init__('tx_node')
        self.get_logger().warn("tx node start")

        self.count = 0

        self.pub = self.create_publisher(String, "msg", 10)
        self.tim = self.create_timer(1.0, self.timer_tx)

    def timer_tx(self):
        msg = String()
        msg.data = f'message number {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTx()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
