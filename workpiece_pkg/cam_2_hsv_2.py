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
cv2.resizeWindow("TrackBars", 400, 400)

cv2.createTrackbar("hue min", "TrackBars", 83,   180, empty)
cv2.createTrackbar("hue max", "TrackBars", 106, 180, empty)
cv2.createTrackbar("sat min", "TrackBars", 83,   255, empty)
cv2.createTrackbar("sat max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("val min", "TrackBars", 75,   255, empty)
cv2.createTrackbar("val max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("kernel", "TrackBars", 5,    15, empty)
cv2.createTrackbar("area_min", "TrackBars", 0,    200, empty)

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
        img_data = np.zeros((img_RGB.shape[0], img_RGB.shape[1], 3), np.uint8)

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

        # img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                      # переводим вх картинку из BGR в HSV
        # img_mask = cv2.inRange(img_HSV, min, max)                           # бинарная картинка (с шумами), где погашены пиксели др цвета
        # edge = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))  # создаем массив для подавления шума
        # mask = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, edge)   

        img_mask_noise = cv2.inRange(img_HSV, M_min, M_max)

        kernel = cv2.getTrackbarPos("kernel", "TrackBars")
        if kernel%2 == 0:
            kernel +=1
        
        edge = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))  # создаем массив для подавления шума
        img_mask_noiseless = cv2.morphologyEx(img_mask_noise, cv2.MORPH_OPEN, edge)

        out = {'cx': -1, 'cy': -1, 'area': -1, 'left': -1, 'right': -1, 'top': -1, 'botton': -1, }

        contours, hierarchy = cv2.findContours(img_mask_noiseless, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # определяем контура
        index = -1                                       # индекс контура
        # area_mas = 0                                     # максимальная площадь
        area_set = cv2.getTrackbarPos("area_min", "TrackBars")
        area_mas = area_set
        N = len(contours)                                # количество контуров
        for i in range(N):                               # найти самый большой контур
            area = cv2.contourArea(contours[i])          # опр площадь каждого отдельного контура
            if area > area_mas:                          # сравниваем площади
                area_mas = area                          # присваиваем площадь
                index = i                                # запоминаем индекс

        if index != -1:
            Mom = cv2.moments(contours[index])               # Моменты (центр тяжести)

            x, y, w, h = cv2.boundingRect(contours[index])   # получаем крайник коор контура

            out.update({'cx' : int(Mom['m10'] / Mom['m00'])})
            out.update({'cy' : int(Mom['m01'] / Mom['m00'])})
            out.update({'area' : cv2.contourArea(contours[index])})
            out.update({'left' : x})
            out.update({'right' : x + w})
            out.update({'top' : y})
            out.update({'botton' : y + h})

        data = out
        
        img_Result = cv2.bitwise_and(img_RGB, img_RGB, mask=img_mask_noiseless)
        if data.get('area') != -1:
            img_Result = cv2.rectangle(img_Result, (data.get('left'), data.get('top')), (data.get('right'), data.get('botton')), (0,255,255), 3)
            img_Result = cv2.circle(img_Result, (data.get('cx'), data.get('cy')), 3, (0,255,255), 1)

        size = 1
        color = (255,255,255)
        thickness = 1
        img_data = cv2.putText(img_data, 'Hue (min, max):', (20, 100), 2, size, color, thickness)
        img_data = cv2.putText(img_data, 'Saturation (min, max):', (20, 150), 2, size, color, thickness)
        img_data = cv2.putText(img_data, 'Value (min, max):', (20, 200), 2, size, color, thickness)
        img_data = cv2.putText(img_data, 'Kernel:', (20, 250), 2, size, color, thickness)
        img_data = cv2.putText(img_data, 'Center (x, y):', (20, 300), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f'Area (>{area_set}):', (20, 350), 2, size, color, thickness)
        img_data = cv2.putText(img_data, 'X (min, max):', (20, 400), 2, size, color, thickness)
        img_data = cv2.putText(img_data, 'Y (min, max):', (20, 450), 2, size, color, thickness)

        img_data = cv2.putText(img_data, f"{self.hsv.get('h_min')}, {self.hsv.get('h_max')}", (420, 100), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{self.hsv.get('s_min')}, {self.hsv.get('s_max')}", (420, 150), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{self.hsv.get('v_min')}, {self.hsv.get('v_max')}", (420, 200), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{kernel}", (420, 250), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{data.get('cx')}, {data.get('cy')}", (420, 300), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{data.get('area')}", (420, 350), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{data.get('left')}, {data.get('right')}", (420, 400), 2, size, color, thickness)
        img_data = cv2.putText(img_data, f"{data.get('top')}, {data.get('botton')}", (420, 450), 2, size, color, thickness)

        
        # -- finish --
        color = (0,0,0)
        img_RGB = cv2.rectangle(img_RGB, (10, 10), (150, 50), color, -1)
        img_HSV = cv2.rectangle(img_HSV, (10, 10), (150, 50), color, -1)
        img_data = cv2.rectangle(img_data, (10, 10), (150, 50), color, -1)
        img_mask_noise = cv2.rectangle(img_mask_noise, (10, 10), (150, 50), color, -1)
        img_mask_noiseless = cv2.rectangle(img_mask_noiseless, (10, 10), (150, 50), color, -1)
        img_Result = cv2.rectangle(img_Result, (10, 10), (150, 50), color, -1)

        color = (255,255,255)
        img_RGB = cv2.putText(img_RGB, 'RGB', (20, 40), 2, 1, color, 2)
        img_HSV = cv2.putText(img_HSV, 'HSV', (20, 40), 2, 1, color, 2)
        img_data = cv2.putText(img_data, 'DATA', (20, 40), 2, 1, color, 2)
        img_mask_noise = cv2.putText(img_mask_noise, 'MASK noise', (20, 40), 2, 1, color, 2)
        img_mask_noiseless = cv2.putText(img_mask_noiseless, 'MASK noiseless', (20, 40), 2, 1, color, 2)
        img_Result = cv2.putText(img_Result, 'RESULT', (20, 40), 2, 1, color, 2)

        img_sc = stackImages (0.8, ([img_RGB,        img_HSV,            img_data ],
                                    [img_mask_noise, img_mask_noiseless, img_Result]))


        cv2.imshow('HSV', img_sc)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CamHSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()