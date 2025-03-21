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

def createArucoMarkerWithBorder(markerSize, markerId, imgSize):
    """
    Создает Aruco маркер с белой рамкой.
    
    :param markerSize: Размер маркера (например, 4x4, 5x5).
    :param markerId: ID маркера.
    :param imgSize: Разрешение изображения маркера.
    :return: Сохраняет маркер в файл.
    """
    # Создаем словарь Aruco маркеров нужного размера
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250 if markerSize == 4 else cv2.aruco.DICT_5X5_250)

    # Генерируем маркер
    markerImage = np.zeros((imgSize, imgSize), dtype=np.uint8)
    cv2.aruco.drawMarker(arucoDict, markerId, imgSize, markerImage, 1)

    # Рассчитываем размер нового изображения с рамкой
    borderSize = imgSize // (markerSize + 2)
    newSize = imgSize + borderSize * 2 + 2

    # Создаем новое изображение с белой рамкой
    newImage = np.ones((newSize, newSize), dtype=np.uint8) * 255
    newImage[borderSize + 1:-borderSize - 1, borderSize + 1:-borderSize - 1] = markerImage
    
    # Добавляем пунктирную линию на крайних пикселях рамки
    for i in range(0, newSize, 4):
        newImage[i:i+2, 0] = 0
        newImage[i:i+2, -1] = 0
        newImage[0, i:i+2] = 0
        newImage[-1, i:i+2] = 0

    # Добавляем текст с ID маркера
    text = f"{markerId}"
    targetTextHeight = imgSize * 0.07  # 7% от высоты изображения
    fontScale = 0.1  # Начальный масштаб шрифта
    thickness = 1 * int(imgSize / 500)
    textSize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale, thickness)[0]

    # Подбираем масштаб шрифта, чтобы высота текста была приблизительно 7% от imgSize
    while textSize[1] < targetTextHeight:
        fontScale += 0.1
        textSize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale, thickness)[0]

    textX = newSize - textSize[0] - int(imgSize * 0.02)  # от правого края
    textY = newSize - int(imgSize * 0.02)  # от нижнего края
    cv2.putText(newImage, text, (textX, textY), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 0, 0), thickness)

    # Сохраняем маркер в файл
    #cv2.imwrite(f"aruco_marker_{markerId}_with_border.png", newImage)
    return newImage

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
        img_sc = stackImages (0.6, ([img_RGB, frame],
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