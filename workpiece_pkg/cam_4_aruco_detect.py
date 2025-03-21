#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

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

def findArucoMarkers(img, markerSize=4, totalMarkers=50, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = cv2.aruco.Dictionary_get(key)
    arucoParam = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)
    if draw:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
    return img, ids, corners

class CamHSV(Node):
    def __init__(self):
        super().__init__('hsv_node')

        self.declare_parameter('topic_name', 'topic_image')
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.bridgeObject = CvBridge()

        self.sub = self.create_subscription(Image, self.topic_name, self.img_cb, 20)

        # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250 if markerSize == 4 else cv2.aruco.DICT_5X5_250)



    def img_cb(self, msg):

        img_RGB = self.bridgeObject.imgmsg_to_cv2(msg)

        height, width, _c = img_RGB.shape
        self.get_logger().info(f'img read. img = {height} x {width}', once = True)

        frame, ids, corners = findArucoMarkers(img_RGB,markerSize=4)
        
        self.get_logger().info(str(ids))

        # if ids == None:
        #     self.get_logger().info("no aruco marker")
        # else:
        #     self.get_logger().info(str(ids))
            # self.get_logger().info('1= ' + str(corners[0][0][0]) + ' 2= ' + str(corners[0][0][1]) + ' 3= ' + str(corners[0][0][2]) + ' 4= ' + str(corners[0][0][3]))
        self.get_logger().info(' ')
        img_sc = stackImages (1.0, ([img_RGB, frame],
                                    [img_RGB, img_RGB]))


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