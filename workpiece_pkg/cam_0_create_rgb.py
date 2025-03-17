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
        self.declare_parameter('screen_on', True)

        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.color = self.get_parameter('color').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.screen_on = self.get_parameter('screen_on').get_parameter_value().bool_value

        self.cv_bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.topic_name, 20)
        self.timer = self.create_timer(0.02, self.timer_cb)

        self.i = 0
        
        self.frame = np.zeros((512, 1024, 3), np.uint8)

        for i in range(255):
            self.frame[0:200, i:i+1 ] = (0,i,255) # bgr red-yellow
            self.frame[0:200, 256+i:256+i+1] = (0,255,255-i) # bgr yellow-green
            self.frame[0:200, 512+i:512+i+1] = (i,255,0) # bgr green-...
            self.frame[0:200, 768+i:768+i+1] = (255,255-i,0) # bgr ...-blue

    def timer_cb(self):

        

        

        img = self.cv_bridge.cv2_to_imgmsg(self.frame)

        self.pub.publish(img)
        # self.get_logger().info(f'image size = [{self.width}, {self.height}], color = {self.color}', throttle_duration_sec=5)
        self.get_logger().info(f'image size = [{self.frame.shape[1]}, {self.frame.shape[0]}], color = {self.color}', throttle_duration_sec=5)

        cv2.imshow('create frame', self.frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    po = CamCreateBlackRect()
    rclpy.spin(po)
    po.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
