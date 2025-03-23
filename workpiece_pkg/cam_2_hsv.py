#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

def empty(a):
    pass

cv2.namedWindow ("TrackBars")
cv2.resizeWindow("TrackBars", 600, 400)

cv2.createTrackbar("hue min", "TrackBars", 0,   180, empty)
cv2.createTrackbar("hue max", "TrackBars", 180, 180, empty)
cv2.createTrackbar("sat min", "TrackBars", 0,   255, empty)
cv2.createTrackbar("sat max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("val min", "TrackBars", 0,   255, empty)
cv2.createTrackbar("val max", "TrackBars", 255, 255, empty)

def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver#  об

class CamHSV(Node):
    def __init__(self):
        super().__init__('hsv_node')

        self.declare_parameter('topic_name', 'topic_image')
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.bridgeObject = CvBridge()

        self.sub = self.create_subscription(Image, self.topic_name, self.img_cb, 20)

        self.hsv = {
            'h_min': 0,
            'h_max': 180,
            's_min': 0,
            's_max': 255,
            'v_min': 0,
            'v_max': 255,
        }

    def img_cb(self, msg):

        img_RGB = self.bridgeObject.imgmsg_to_cv2(msg)

        height, width, _c = img_RGB.shape
        self.get_logger().info(f'img read. img = {height} x {width}', once = True)

        img_HSV = cv2.cvtColor(img_RGB, cv2.COLOR_BGR2HSV)        # перевод из RGB в HSV

        self.hsv.update({'h_min' : cv2.getTrackbarPos("hue min", "TrackBars")})
        self.hsv.update({'h_max' : cv2.getTrackbarPos("hue max", "TrackBars")})
        self.hsv.update({'s_min' : cv2.getTrackbarPos("sat min", "TrackBars")})
        self.hsv.update({'s_max' : cv2.getTrackbarPos("sat max", "TrackBars")})
        self.hsv.update({'v_min' : cv2.getTrackbarPos("val min", "TrackBars")})
        self.hsv.update({'v_max' : cv2.getTrackbarPos("val max", "TrackBars")})

        M_min = np.array([self.hsv.get('h_min'), self.hsv.get('s_min'), self.hsv.get('v_min')])
        M_max = np.array([self.hsv.get('h_max'), self.hsv.get('s_max'), self.hsv.get('v_max')])

        img_mask = cv2.inRange(img_HSV, M_min, M_max)
        img_Result = cv2.bitwise_and(img_RGB, img_RGB, mask=img_mask)

        img_RGB = cv2.rectangle(img_RGB, (10, 10), (150, 50), (0,0,0), -1)
        img_HSV = cv2.rectangle(img_HSV, (10, 10), (150, 50), (0,0,0), -1)
        img_mask = cv2.rectangle(img_mask, (10, 10), (150, 50), (0,0,0), -1)
        img_Result = cv2.rectangle(img_Result, (10, 10), (150, 50), (0,0,0), -1)

        img_RGB = cv2.putText(img_RGB, 'RGB', (20, 40), 2, 1, (255,255,255), 2)
        img_HSV = cv2.putText(img_HSV, 'HSV', (20, 40), 2, 1, (255,255,255), 2)
        img_mask = cv2.putText(img_mask, 'MASK', (20, 40), 2, 1, (255,255,255), 2)
        img_Result = cv2.putText(img_Result, 'RESULT', (20, 40), 2, 1, (255,255,255), 2)

        img_sc = stackImages (1.0, ([img_RGB,  img_HSV],
                                    [img_mask, img_Result]))


        cv2.imshow('camera', img_sc)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CamHSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()