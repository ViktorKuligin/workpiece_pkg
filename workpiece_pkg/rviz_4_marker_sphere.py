#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class RvizMarkerSphere(Node):
    def __init__(self):
        super().__init__('marker_sphere_node')
        self.get_logger().warn("marker sphere node start")

        self.pub = self.create_publisher(Marker, "marker_sphere", 10)
        self.tim = self.create_timer(1.0, self.marker_sphere_timers)

    def marker_sphere_timers(self):
        msg = Marker()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.type = Marker.SPHERE
        msg.pose.position = Point(x = 1.0, y = 1.0, z = 1.0)
        msg.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
        msg.scale = Vector3(x = 1.0, y = 1.0, z = 1.0)
        msg.color = ColorRGBA(r = 0.0, g = 0.0, b = 1.0, a = 1.0)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RvizMarkerSphere()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()