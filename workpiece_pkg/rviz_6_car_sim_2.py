#!/usr/bin/env python3
import rclpy
import numpy as np
from tf_transformations import quaternion_from_euler # sudo apt install ros-humble-tf-transformations

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Quaternion, PoseStamped, Twist, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

color = {
    # r - red, g - green, b - blue, a - transparency
    'red': ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0),
    'blue': ColorRGBA(r = 0.0, g = 0.0, b = 1.0, a = 1.0),
    'green': ColorRGBA(r = 0.0, g = 1.0, b = 0.0, a = 1.0),
    'white': ColorRGBA(r = 1.0, g = 1.0, b = 1.0, a = 1.0),
}

class CarSimulator(Node):
    def __init__(self):
        super().__init__('car_sim_node')
        self.get_logger().warn("car simulator node start")

        # self.declare_parameter('topic_name', '/turtle1/cmd_vel')
        self.declare_parameter('topic_name', 'direction_mode')
        self.declare_parameter('color', 'green')
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.car_xx = 0.0
        self.car_yy = 0.0
        self.car_zz = 0.25
        self.car_angle_z = 0.0
        self.car_angle_w = 0.0
        self.car_color = color.get(self.color)

        # self.pub_arrow = self.create_publisher(PoseStamped, self.topic_name, 10)
        # self.sub = self.create_subscription(Twist, self.topic_name, self.car_cb, 10)
        self.sub = self.create_subscription(Pose, self.topic_name, self.car_cb, 10)
        self.pub_body = self.create_publisher(Marker, "car_body", 10)
        self.pub_arrow = self.create_publisher(Marker, "car_arrow", 10)

        # self.tim = self.create_timer(1.0, self.marker_sphere_timers)

    def marker_sphere_timers(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.type = Marker.CUBE
        msg.pose.position = Point(x = self.car_xx, y = self.car_yy, z = self.car_zz)
        q = quaternion_from_euler(0.0, 0.0, self.car_angle * np.pi / 180)
        msg.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        msg.scale = Vector3(x = 1.0, y = 0.75, z = 0.5)
        msg.color = self.car_color
        self.pub_body.publish(msg)

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.type = Marker.ARROW
        msg.pose.position = Point(x = self.car_xx, y = self.car_yy, z = self.car_zz)
        q = quaternion_from_euler(0.0, 0.0, self.car_angle * np.pi / 180)
        msg.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        msg.scale = Vector3(x = 1.0, y = 0.25, z = 0.25)
        msg.color = self.car_color
        self.pub_arrow.publish(msg)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 0.0
        msg.pose.orientation.z = 1.0

    def car_cb(self, msg: Pose):

        # self.car_angle = self.car_angle + msg.angular.z * 0.5

        # self.car_xx = self.car_xx + msg.linear.x * 0.01 * np.cos(self.car_angle * np.pi / 180)
        # self.car_yy = self.car_yy + msg.linear.x * 0.01 * np.sin(self.car_angle * np.pi / 180)

        # self.get_logger().warn(f'x={round(self.car_xx,3)}, y={round(self.car_yy,3)}, angle = {round(self.car_angle,3)}')

        self.car_xx = msg.position.x
        self.car_yy = msg.position.y
        self.car_angle_z = msg.orientation.z
        self.car_angle_w = msg.orientation.w
        
        msg_body = Marker()
        msg_body.header.stamp = self.get_clock().now().to_msg()
        msg_body.header.frame_id = 'map'
        msg_body.type = Marker.CUBE
        msg_body.pose.position = Point(x = self.car_xx, y = self.car_yy, z = self.car_zz)
        # q = quaternion_from_euler(0.0, 0.0, self.car_angle * np.pi / 180)
        # msg_body.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        msg_body.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = self.car_angle_z, w = self.car_angle_w)
        msg_body.scale = Vector3(x = 1.0, y = 0.75, z = 0.5)
        msg_body.color = self.car_color
        self.pub_body.publish(msg_body)

        msg_arrow = Marker()
        msg_arrow.header.stamp = self.get_clock().now().to_msg()
        msg_arrow.header.frame_id = 'map'
        msg_arrow.type = Marker.ARROW
        msg_arrow.pose.position = Point(x = self.car_xx, y = self.car_yy, z = self.car_zz)
        # q = quaternion_from_euler(0.0, 0.0, self.car_angle * np.pi / 180)
        # msg_arrow.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        msg_arrow.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = self.car_angle_z, w = self.car_angle_w)
        msg_arrow.scale = Vector3(x = 1.0, y = 0.25, z = 0.25)
        msg_arrow.color = self.car_color
        self.pub_arrow.publish(msg_arrow)

def main(args=None):
    rclpy.init(args=args)
    node = CarSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()