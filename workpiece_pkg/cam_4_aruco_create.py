#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge


class CamArucoCreate(Node):
    def __init__(self):
        super().__init__('aruco_create_node')
        self.get_logger().warn('aruco create node start')

        self.declare_parameter('topic_name', 'aruco_image')
        self.declare_parameter('arucoId', 0)
        self.declare_parameter('arucoSize', 4)
        self.declare_parameter('imagePixelSize', 50)
        self.declare_parameter('continuosly', False)
        self.declare_parameter('timerStep', 0.1)
        self.declare_parameter('screen', True)

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.arucoId = self.get_parameter('arucoId').get_parameter_value().integer_value
        self.arucoSize = self.get_parameter('arucoSize').get_parameter_value().integer_value
        self.imagePixelSize = self.get_parameter('imagePixelSize').get_parameter_value().integer_value
        self.continuosly = self.get_parameter('continuosly').get_parameter_value().bool_value
        self.timerStep = self.get_parameter('timerStep').get_parameter_value().double_value
        self.screen = self.get_parameter('screen').get_parameter_value().bool_value

        if self.screen:
            self.get_logger().info(f'arocoId = {self.arucoId}')
            self.get_logger().info(f'arucoSize = {self.arucoSize}')
            self.get_logger().info(f'imagePixelSize = {self.imagePixelSize}')

        self.cv_bridge = CvBridge()

        self.pub = self.create_publisher(Image, self.topic_name, 20)
        self.tim = self.create_timer(self.timerStep, self.timer_create_aruco_marker)

    def timer_create_aruco_marker(self):

        img_aruco = self.createArucoMarkerImg(self.arucoId, self.arucoSize, self.imagePixelSize)

        if self.continuosly:
            self.get_logger().info(str(self.arucoId))
            self.arucoId += 1
            if self.arucoId == 250:
                self.arucoId = 0

        img = self.cv_bridge.cv2_to_imgmsg(img_aruco)
        self.pub.publish(img)

        cv2.imshow('camera', img_aruco)
        cv2.waitKey(1)

    def createArucoMarkerImg(self, id, arucoMarkerSize, pixelSize):

        if arucoMarkerSize < 4: arucoMarkerSize = 4
        if arucoMarkerSize > 7: arucoMarkerSize = 7

        markerDict = {
            4 : cv2.aruco.DICT_4X4_250,
            5 : cv2.aruco.DICT_5X5_250,
            6 : cv2.aruco.DICT_6X6_250,
            7 : cv2.aruco.DICT_7X7_250,
        }

        matrixBlackSize = arucoMarkerSize + 2
        matrixBlack = np.zeros((matrixBlackSize, matrixBlackSize), dtype=np.uint8)
        arucoDict = cv2.aruco.Dictionary_get(markerDict.get(arucoMarkerSize))
        matrixAruco = cv2.aruco.drawMarker(arucoDict, id, matrixBlackSize, matrixBlack, 1)

        matrixWhiteSize = matrixBlackSize + 2
        matrixWhite = np.ones((matrixWhiteSize, matrixWhiteSize), dtype=np.uint8) * 255
        for i in range(matrixBlackSize):
            for j in range(matrixBlackSize):
                matrixWhite[i+1][j+1] = matrixAruco[i][j]

        imgArucoSize = matrixWhiteSize * pixelSize
        imgAruco = np.zeros((imgArucoSize, imgArucoSize), dtype=np.uint8)
        for i in range(matrixWhiteSize):
            for j in range(matrixWhiteSize):
                if matrixWhite[i][j] == 255:
                    imgAruco[i * pixelSize : (i + 1) * pixelSize, j * pixelSize : (j + 1) * pixelSize] = 255

        return imgAruco


def main(args=None):
    rclpy.init(args=args)
    node = CamArucoCreate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()