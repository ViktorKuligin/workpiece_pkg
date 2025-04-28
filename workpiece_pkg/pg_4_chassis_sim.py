#!/usr/bin/env python3
import rclpy
import numpy as np
import pygame as pg      # pip3 install pygame

from sensor_msgs.msg import Joy    # ros2 launch teleop_twist_joy teleop-launch.py
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

pg.init()
pg.display.set_caption('Direction')
screen = pg.display.set_mode((400,400))
screen.fill((0,0,0))
pg.draw.rect(screen, (255,255,0), (0,0,400,400), 5)
pg.display.update()

font = pg.font.Font(None, 50)

wb = 90
thickness = 5

button_color = {
    'on': (0, 255, 0),
    'off': (255, 0, 0)
}
button_location = {
    '1': (100-int(wb/2), 300-int(wb/2), wb, wb),
    '2': (200-int(wb/2), 300-int(wb/2), wb, wb),
    '3': (300-int(wb/2), 300-int(wb/2), wb, wb),
    '4': (100-int(wb/2), 200-int(wb/2), wb, wb),
    '5': (200-int(wb/2), 200-int(wb/2), wb, wb),
    '6': (300-int(wb/2), 200-int(wb/2), wb, wb),
    '7': (100-int(wb/2), 100-int(wb/2), wb, wb),
    '8': (200-int(wb/2), 100-int(wb/2), wb, wb),
    '9': (300-int(wb/2), 100-int(wb/2), wb, wb),
}
button_value = {
    '1': (-0.7071, 0.7071),
    '2': (-1.0, 0.0),
    '3': (-0.7071, -0.7071),
    '4': (0.0, 1.0),
    '5': (0.0, 0.0),
    '6': (0.0, -1.0),
    '7': (0.7071, 0.7071),
    '8': (1.0, 0.0),
    '9': (0.7071, -0.7071),
}
button_scancode = {
    89: '1',
    90: '2',
    91: '3',
    92: '4',
    93: '5',
    94: '6',
    95: '7',
    96: '8',
    97: '9',
}

def button_draw(button_name, status):
    color = button_color.get(status)
    array = button_location.get(button_name)
    pg.draw.rect(screen, color, array, thickness)
    img = font.render(button_name, True, color)
    screen.blit(img, (array[0] + 10, array[1] + 20))

class JoyGreen(Node):
    def __init__(self):
        super().__init__('joy_green')
        self.get_logger().warn("joy green start")

        self.vel = 0.5
        self.button_name = ''

        self.pub = self.create_publisher(Vector3, 'direction', 10)
        self.keyboard = self.create_timer(0.01, self.keyboard_timer)

        for i in range(1,10):
            button_draw(str(i), 'off')
        pg.display.update()

    def keyboard_timer(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
            if event.type == pg.KEYDOWN:
                if event.scancode in button_scancode:
                    self.button_name = button_scancode.get(event.scancode)
                    button_draw(self.button_name, 'on')
                    msg = Vector3()
                    msg.x = button_value.get(self.button_name)[0] * self.vel
                    msg.y = button_value.get(self.button_name)[1] * self.vel
                    self.get_logger().info(f'x={msg.x}, y={msg.y}')
                    self.pub.publish(msg)
                    # self.button_name = ''

            if event.type == pg.KEYUP:
                for i in range(1,10):
                    button_draw(str(i), 'off')

        pg.display.update()

def main(args=None):
    rclpy.init(args=args)
    node = JoyGreen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()