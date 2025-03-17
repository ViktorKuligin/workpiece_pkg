#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Range

class RvizRange(Node):
    def __init__(self):
        super().__init__('range_node')
        self.get_logger().warn("range node start")

        self.distance = 2.0
        self.distance_min = 0.3
        self.distance_max = 4.5
        self.direction = 1.0

        self.pub = self.create_publisher(Range, "my_range", 10)
        self.tim = self.create_timer(0.01, self.my_timers)

    def my_timers(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.radiation_type = Range.INFRARED       #ULTRASOUND
        msg.field_of_view = 0.52
        msg.min_range = self.distance_min
        msg.max_range = self.distance_max
        msg.range = self.distance

        self.pub.publish(msg)
        self.get_logger().info(str(msg))

        self.distance = self.distance + 0.01 * self.direction

        if self.distance > self.distance_max:
            self.direction = -1.0
        elif self.distance < self.distance_min:
            self.direction = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = RvizRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()