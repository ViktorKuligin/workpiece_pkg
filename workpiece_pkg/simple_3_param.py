#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

class Parameter(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.get_logger().warn("parameter node start")

        self.count = 0

        self.declare_parameter('topic_name', 'msg')
        self.declare_parameter('timer_period', 1.0)
        self.declare_parameter('print_available', True)
        self.declare_parameter('fix_data', 12)

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.print_available = self.get_parameter('print_available').get_parameter_value().bool_value
        self.fix_data = self.get_parameter('fix_data').get_parameter_value().integer_value

        self.pub = self.create_publisher(String, self.topic_name, 10)
        self.tim = self.create_timer(self.timer_period, self.timer_tx)

    def timer_tx(self):
        msg = String()
        msg.data = f'message number {self.count}'
        self.pub.publish(msg)

        if self.print_available:
            text = f'fix = {self.fix_data}, msg = {msg.data}'
            self.get_logger().info(text)
        else:
            self.get_logger().warm('not allowed')

        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Parameter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()