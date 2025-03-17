#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Vector3, Quaternion
from sensor_msgs.msg import Range

class RvizPose(Node):
    def __init__(self):
        super().__init__('pose_node')
        self.get_logger().warn("pose node start")

        self.degree_1 = 0.0
        self.degree_2 = 0.0
        self.degree_3 = 0.0

        self.alpha_pose = 90.0
        self.alpha_orient = 0.0
        self.zz = 0.0
        self.znak = 1.0

        self.pub_static = self.create_publisher(PoseStamped, "pose_static", 10)
        self.pub_dynamic_1 = self.create_publisher(PoseStamped, "pose_dynamic_1", 10)
        self.pub_dynamic_2 = self.create_publisher(PoseStamped, "pose_dynamic_2", 10)
        self.pub_dynamic_3 = self.create_publisher(PoseStamped, "pose_dynamic_3", 10)

        self.tim_static = self.create_timer(0.05, self.timers_static)
        self.tim_dynamic_1 = self.create_timer(0.05, self.timers_dynamic_1)
        self.tim_dynamic_2 = self.create_timer(0.05, self.timers_dynamic_2)
        self.tim_dynamic_3 = self.create_timer(0.05, self.timers_dynamic_3)

    def timers_static(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = Point(x=3.0, y=3.0, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.pub_static.publish(msg)

    def timers_dynamic_1(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = -3 + np.cos(self.degree_1 * np.pi / 180) * 1
        msg.pose.position.y =  3 + np.sin(self.degree_1 * np.pi / 180) * 1
        msg.pose.position.z = 0.0
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.pub_dynamic_1.publish(msg)

        # text = f'deg = {self.degree_1}, pos = {msg.pose.position}'
        # self.get_logger().info(text)

        self.degree_1 += 1
        if self.degree_1 == 360:
            self.degree_1 = 0

    def timers_dynamic_2(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = Point(x=-3.0, y=-3.0, z=0.0)

        q = self.quaternion_from_euler(0.0, 0.0, self.degree_2 * np.pi / 180)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub_dynamic_2.publish(msg)

        # text = f'deg = {self.degree_2}, pos = {msg.pose.orientation}'
        # self.get_logger().info(text)

        self.degree_2 += 1
        if self.degree_2 == 360:
            self.degree_2 = 0

    def timers_dynamic_3(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x =  3 + np.cos(self.degree_3 * np.pi / 180) * 1
        msg.pose.position.y = -3 + np.sin(self.degree_3 * np.pi / 180) * 1
        msg.pose.position.z = 0.0

        q = self.quaternion_from_euler(0.0, 0.0, (self.degree_3 + 90) * np.pi / 180)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub_dynamic_3.publish(msg)

        # text = f'deg = {self.degree_1}, pos = {msg.pose.position}'
        # self.get_logger().info(text)

        self.degree_3 += 1
        if self.degree_3 == 360:
            self.degree_3 = 0

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)

        return[0.0, 0.0, sy, cy]


def main(args=None):
    rclpy.init(args=args)
    node = RvizPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()