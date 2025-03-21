#!/usr/bin/env python3
import cv2
import rclpy

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class CamReaderUSB(Node):
    def __init__(self):
        super().__init__('cam_reader')
        self.get_logger().warn("cam reader start")

        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.cv_bridge = CvBridge()

        self.pub = self.create_publisher(Image, 'topic_image', 20)
        self.timer = self.create_timer(0.02, self.timer_cb)

        self.i = 0

    def timer_cb(self):
        success, frame = self.camera.read()

        # frame = cv2.resize(frame, (820,640), interpolation=cv2.INTER_CUBIC)

        if success == True:
            img = self.cv_bridge.cv2_to_imgmsg(frame)
            self.pub.publish(img)
            self.get_logger().info(f'number = {self.i}', throttle_duration_sec=1)

            self.i += 1
            if self.i > 1000:
                self.i = 0
        else:
            self.get_logger().info(f'fail')


def main(args=None):
    rclpy.init(args=args)
    node = CamReaderUSB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
