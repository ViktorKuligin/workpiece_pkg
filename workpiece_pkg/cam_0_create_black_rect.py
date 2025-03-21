#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class CamCreateBlackRect(Node):
    def __init__(self):
        super().__init__('create_img')
        self.get_logger().warn("create img node start")
        
        self.colorDict = {
            'black': (0,0,0),
            'white': (255,255,255),
            'red': (255,0,0),
            'blue': (0,0,255),
            'green': (0,255,0),
        }

        self.declare_parameter('height', 480)
        self.declare_parameter('width', 640)
        self.declare_parameter('color', 'black')
        self.declare_parameter('topic_name', 'topic_image')

        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.color = self.get_parameter('color').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.cv_bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.topic_name, 20)
        self.timer = self.create_timer(0.02, self.timer_cb)

        self.i = 0

    def timer_cb(self):
        if self.height < 240:
            self.height = 240

        if self.width < 320:
            self.width = 320

        frame = np.zeros((self.height, self.width, 3), np.uint8)
        frame[:] = self.colorDict.get(self.color)

        img = self.cv_bridge.cv2_to_imgmsg(frame)

        self.pub.publish(img)
        self.get_logger().info(f'image size = [{self.width}, {self.height}], color = {self.color}', throttle_duration_sec=5)
        
        # cv2.imshow('create frame', frame)
        # cv2.waitKey(1)

        # success, frame = self.camera.read()

        # # frame = cv2.resize(frame, (820,640), interpolation=cv2.INTER_CUBIC)

        # if success == True:
        #     img = self.cv_bridge.cv2_to_imgmsg(frame)
        #     self.pub.publish(img)
        #     self.get_logger().info(f'number = {self.i}', throttle_duration_sec=1)

        #     self.i += 1
        # else:
        #     self.get_logger().info(f'fail')


def main(args=None):
    rclpy.init(args=args)
    node = CamCreateBlackRect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
