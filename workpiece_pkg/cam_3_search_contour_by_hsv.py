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
    return ver

def find_con_by_HSV_max(img, color, kernel):
    HSVmin = color[0], color[2], color[4]                               # выделяем данные из списка по мин значениям
    HSVmax = color[1], color[3], color[5]                               # выделяем данные из списка по мах значениям
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                      # переводим вх картинку из BGR в HSV
    img_mask = cv2.inRange(img_HSV, HSVmin, HSVmax)                     # бинарная картинка (с шумами), где погашены пиксели др цвета
    edge = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))  # создаем массив для подавления шума
    mask = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, edge)             # бинарная картинка без шумов
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # определяем контура
    index = -1                                       # индекс контура
    area_mas = 0                                     # максимальная площадь
    N = len(contours)                                # количество контуров
    for i in range(N):                               # найти самый большой контур
        area = cv2.contourArea(contours[i])          # опр площадь каждого отдельного контура
        if area > area_mas:                          # сравниваем площади
            area_mas = area                          # присваиваем площадь
            index = i                                # запоминаем индекс
    Mom = cv2.moments(contours[index])               # Моменты (центр тяжести)
    Cx = int(Mom['m10'] / Mom['m00'])                # ось Х
    Cy = int(Mom['m01'] / Mom['m00'])                # ось Y
    Area = cv2.contourArea(contours[index])
    x, y, w, h = cv2.boundingRect(contours[index])   # получаем крайник коор контура
    U = y                                            # верх
    D = y + h                                        # низ
    L = x                                            # лево
    R = x + w                                        # право
    sizes = ( Cx, Cy, Area, x, y, x + w, y + h )             # пакет данных
    return sizes                                     # отправляем пакет

def find_con_by_HSV_max_dict(img, hsv, kernel):

    min = hsv.get('h_min'), hsv.get('s_min'), hsv.get('v_min')
    max = hsv.get('h_max'), hsv.get('s_max'), hsv.get('v_max')

    out = {
        'cx': -1,
        'cy': -1,
        'area': -1,
        'left': -1,
        'right': -1,
        'up': -1,
        'down': -1,
    }

    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                      # переводим вх картинку из BGR в HSV
    img_mask = cv2.inRange(img_HSV, min, max)                           # бинарная картинка (с шумами), где погашены пиксели др цвета
    edge = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))  # создаем массив для подавления шума
    mask = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, edge)             # бинарная картинка без шумов
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # определяем контура
    index = -1                                       # индекс контура
    area_mas = 0                                     # максимальная площадь
    N = len(contours)                                # количество контуров
    for i in range(N):                               # найти самый большой контур
        area = cv2.contourArea(contours[i])          # опр площадь каждого отдельного контура
        if area > area_mas:                          # сравниваем площади
            area_mas = area                          # присваиваем площадь
            index = i                                # запоминаем индекс

    if index != -1:
        Mom = cv2.moments(contours[index])               # Моменты (центр тяжести)

        out.update({'cx' : int(Mom['m10'] / Mom['m00'])})
        out.update({'cy' : int(Mom['m01'] / Mom['m00'])})

        out.update({'area' : cv2.contourArea(contours[index])})

        x, y, w, h = cv2.boundingRect(contours[index])   # получаем крайник коор контура

        out.update({'left' : x})
        out.update({'right' : x + w})
        out.update({'up' : y})
        out.update({'down' : y + h})

    return out                                     # отправляем пакет

def find_con_by_HSV_all(img, color, kernel):         # на доработке
    # пример вызова функции
    # col = ( 0, 7, 5, 255, 116, 255)                # цвет в формате HSV
    # dots = mm.find_con_by_HSV_all(img, col, 3)     # получаем массив с 4-мя коор в строчку

    HSVmin = color[0], color[2], color[4]                               # выделяем данные из списка по мин значениям
    HSVmax = color[1], color[3], color[5]                               # выделяем данные из списка по мах значениям
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                      # переводим вх картинку из BGR в HSV
    img_mask = cv2.inRange(img_HSV, HSVmin, HSVmax)                     # бинарная картинка (с шумами), где погашены пиксели др цвета
    edge = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))  # создаем массив для подавления шума
    mask = cv2.morphologyEx(img_mask, cv2.MORPH_OPEN, edge)             # бинарная картинка без шумов
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # определяем контура
    N = len(contours)                               # количество контуров
    mas_sizes = np.zeros((N, 7), int)               # создаем массив под коордиаты контуров
    for i in range(N):                              # перебираем контура
        Mom = cv2.moments(contours[i])              # Моменты (центр тяжести)
        Cx = int(Mom['m10'] / Mom['m00'])           # ось Х
        Cy = int(Mom['m01'] / Mom['m00'])           # ось Y
        Area = cv2.contourArea(contours[i])         # площадь контура
        x, y, w, h = cv2.boundingRect(contours[i])  # получаем крайник коор контура
        U = y                                       # крайнее верхнее
        D = y + h                                   # крайнее нижнее
        L = x                                       # крайнее левое
        R = x + w                                   # крайнее правое
        mas_sizes[i, 0] = Cx                        # заполняем массив - центр Х
        mas_sizes[i, 1] = Cy                        # заполняем массив - центр Y
        mas_sizes[i, 2] = Area                      # заполняем массив - площадь
        mas_sizes[i, 3] = x                         # заполняем массив - крайнее левое
        mas_sizes[i, 4] = y                         # заполняем массив - крайнее правое
        mas_sizes[i, 5] = x + w                         # заполняем массив - крайнее верхнее
        mas_sizes[i, 6] = y + h                         # заполняем массив - крайнее нижнее
        # пакет [ [Cx0, Cy0, area0, left0, right0, up0, down0]
        #         [Cx1, Cy1, area1, left1, right1, up1, down1]
        #         [Cx2, Cy2, area2, left2, right2, up2, down2] ]
    return mas_sizes                                # отправляем пакет [[Cx, Cy, area, left, right, up, down] [...]]

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

        self.hsv.update({'h_min' : 9})
        self.hsv.update({'h_max' : 30})
        self.hsv.update({'s_min' : 91})
        self.hsv.update({'s_max' : 225})
        self.hsv.update({'v_min' : 216})
        self.hsv.update({'v_max' : 255})

        data = find_con_by_HSV_max_dict(img_RGB, self.hsv, 5)

        self.get_logger().info(str(data.get('area')))

        img_Result = img_RGB.copy()

        if data.get('area') != -1:
            img_Result = cv2.rectangle(img_Result, (data.get('left'), data.get('up')), (data.get('right'), data.get('down')), (0,0,0), 1)
            img_Result = cv2.circle(img_Result, (data.get('cx'), data.get('cy')), 3, (0,0,0), 1)
            text = f'area = {data.get("area")}'
            # img = cv2.putText(img, text, (data.get('cx') + 5, data.get('cy') - 5), 1, 1, (250,250,250), 1)


        img_RGB = cv2.rectangle(img_RGB, (10, 10), (150, 50), (0,0,0), -1)
        # img_HSV = cv2.rectangle(img_HSV, (10, 10), (150, 50), (0,0,0), -1)
        # img_mask = cv2.rectangle(img_mask, (10, 10), (150, 50), (0,0,0), -1)
        img_Result = cv2.rectangle(img_Result, (10, 10), (150, 50), (0,0,0), -1)

        img_RGB = cv2.putText(img_RGB, 'RGB', (20, 40), 2, 1, (255,255,255), 2)
        # img_HSV = cv2.putText(img_HSV, 'HSV', (20, 40), 2, 1, (255,255,255), 2)
        # img_mask = cv2.putText(img_mask, 'MASK', (20, 40), 2, 1, (255,255,255), 2)
        img_Result = cv2.putText(img_Result, 'RESULT', (20, 40), 2, 1, (255,255,255), 2)

        img_sc = stackImages (1.0, ([img_RGB,  img_RGB],
                                    [img_RGB, img_Result]))


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