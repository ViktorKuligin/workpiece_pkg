#!/usr/bin/env python3
import cv2
import rclpy

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class CamPaint(Node):
    def __init__(self):
        super().__init__('opencv_read_node')

        self.bridgeObject = CvBridge()

        self.sub = self.create_subscription(Image, 'topic_image', self.img_cb, 20)

    def img_cb(self, msg):

        img = self.bridgeObject.imgmsg_to_cv2(msg)
        height, width, _c = img.shape
        self.get_logger().info(f'img read. img = {height} x {width}', once = True)

        # text

        img = cv2.putText(img, "text", (50, 50), 1, 1, (250,250,250), 1)

        img = cv2.rectangle(img, (100, 50), (250, 150), (250,250,250), 1)

        img = cv2.line(img, (200, 100), (450, 350), (250,250,250), 1)

        img = cv2.circle(img, (300,300), 15, (255,0,0), -1)

        text = f'img size = [{img.shape}]'
        img = cv2.putText(img, text, (50, 350), 1, 1, (250,250,250), 1)

        cv2.imshow('camera', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    po = CamPaint()
    rclpy.spin(po)
    po.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()