#!/usr/bin/env python3
import rclpy
import numpy as np
from tf_transformations import quaternion_from_euler # sudo apt install ros-humble-tf-transformation

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class RvizMarkerSphere(Node):
    def __init__(self):
        super().__init__('marker_sphere_node')
        self.get_logger().warn("marker sphere node start")

        self.pub_st = self.create_publisher(Marker, "marker_sphere_static", 10)
        self.pub_dy = self.create_publisher(Marker, "marker_sphere_dynamic", 10)

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
        self.pub_st.publish(msg)

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.type = Marker.CUBE
        msg.pose.position = Point(x = 0.0, y = 0.0, z = 0.0)
        q = quaternion_from_euler(0.0, 0.0, 45.0 * np.pi / 180)
        # msg.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        msg.scale = Vector3(x = 1.0, y = 1.0, z = 1.0)
        msg.color = ColorRGBA(r = 0.0, g = 0.0, b = 1.0, a = 1.0)
        self.pub_dy.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RvizMarkerSphere()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()