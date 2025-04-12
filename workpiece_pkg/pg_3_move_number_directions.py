#!/usr/bin/env python3
import rclpy
import numpy as np
import pygame as pg

from sensor_msgs.msg import Joy    # ros2 launch teleop_twist_joy teleop-launch.py
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

pg.init()
screen = pg.display.set_mode((500,500))
pg.display.update()

color_on = (0, 255, 0)
color_off = (255, 0, 0)

button_color_status = {
    'on': (0, 255, 0),
    'off': (255, 0, 0)
}

font = pg.font.Font(None, 24)

cx = 30
wb = 70 # размер кнопки
TH1 = 5 # толщина контура прямоугольника при нажатии
dl = 20

# masb1 = np.array([cx+dl,  70, wb, wb])



location = {
    '1': (50, 250, wb, wb),
    '2': (150, 250, wb, wb),
    '3': (250, 250, wb, wb),
    '4': (50, 150, wb, wb),
    '5': (150, 150, wb, wb),
    '6': (250, 150, wb, wb),
    '7': (50, 50, wb, wb),
    '8': (150, 50, wb, wb),
    '9': (250, 50, wb, wb),
}

def button_draw(button_name, status):
    color = button_color_status.get(status)
    array = location.get(button_name)
    pg.draw.rect(screen, color, array, TH1)
    img = font.render(button_name, True, color)
    screen.blit(img, (array[0] + 10, array[1] + 20))
    

class JoyGreen(Node):
    def __init__(self):
        super().__init__('joy_green')
        self.get_logger().warn("joy green start")

        self.xx = 0.0
        self.yy = 0.0

        self.vel = 0.5

        self.button_name = ''

        # self.font1 = pg.font.Font(None, 24) # размер текста для надписи кнопки
        # self.colorTB0 = (255, 0, 0)      # цвет текста - кнопка не нажата
        # self.colorTB1 = (0, 255, 0)      # цвет текста - кнопка нажата

        self.cx = 30
        self.wb = 70 # размер кнопки
        self.TH1 = 5 # толщина контура прямоугольника при нажатии
        self.dl = 20
        self.masb1 = np.array([self.cx+self.dl,  70, self.wb, self.wb])  # кнопка 1

        # self.pg.init()

        self.joy_red_button = {
            0: "CROSS",
            1: "CIRCLE",
            2: "-----",
            3: "SQUARE",
            4: "TRIANGLE",
            5: "-----",
            6: "L1",
            7: "R1",
            8: "L2",
            9: "R2",
            10: "SELECT",
            11: "START",
            12: "MODE",    # do not use
        }
        self.joy_green_button = {
            0: "TRIANGLE",
            1: "CIRCLE",
            2: "CROSS",
            3: "SQUARE",
            4: "L1",
            5: "R1",
            6: "L2",
            7: "R2",
            8: "SELECT",
            9: "START",
            10: "-----",
            11: "-----",
            12: "-----",
        }
        self.joy_red_axes = {
            '0': 'pad_left, rotate: left',
            '-0': 'pad_left, rotate: right',
            '1': 'pad_left, rotate: up',
            '-1': 'pad_left, rotate: down',
            '2': 'pad_right, rotate: left',
            '-2': 'pad_right, rotate: right',
            '3': 'pad_right, rotate: up',
            '-3': 'pad_right, rotate: down',
            '4': '-----',
            '5': '-----',
            '6': 'arraw_left',
            '-6': 'arraw_right',
            '7': 'arraw_up',
            '-7': 'arraw_down',
        }
        self.joy_green_axes = {
            '0': 'pad_left, rotate: left',
            '-0': 'pad_left, rotate: right',
            '1': 'pad_left, rotate: up',
            '-1': 'pad_left, rotate: down',
            '2': 'pad_right, rotate: left',
            '-2': 'pad_right, rotate: right',
            '3': 'pad_right, rotate: up',
            '-3': 'pad_right, rotate: down',
            '4': 'arraw_left',
            '-4': 'arraw_right',
            '5': 'arraw_up',
            '-5': 'arraw_down',
        }
        self.dict_scancode = {
            4:  'a',           # pg.K_a
            5:  'b',           # pg.K_b
            6:  'c',           # pg.K_c
            7:  'd',           # pg.K_d
            8:  'e',           # pg.K_e
            9:  'f',           # pg.K_f
            10: 'g',           # pg.K_g
            11: 'h',           # pg.K_h
            12: 'i',           # pg.K_i
            13: 'j',           # pg.K_j
            14: 'k',           # pg.K_k
            15: 'l',           # pg.K_l
            16: 'm',           # pg.K_m
            17: 'n',           # pg.K_n
            18: 'o',           # pg.K_o
            19: 'p',           # pg.K_p
            20: 'q',           # pg.K_q
            21: 'r',           # pg.K_r
            22: 's',           # pg.K_s
            23: 't',           # pg.K_t
            24: 'u',           # pg.K_u
            25: 'v',           # pg.K_v
            26: 'w',           # pg.K_w
            27: 'x',           # pg.K_x
            28: 'y',           # pg.K_y
            29: 'z',           # pg.K_z
            30: '1',           # pg.K_1
            31: '2',           # pg.K_2
            32: '3',           # pg.K_3
            33: '4',           # pg.K_4
            34: '5',           # pg.K_5
            35: '6',           # pg.K_6
            36: '7',           # pg.K_7
            37: '8',           # pg.K_8
            38: '9',           # pg.K_9
            39: '0',           # pg.K_0
            40: 'Enter',       # pg.K_RETURN
            41: 'ESC',         # pg.K_ESCAPE
            42: 'BackSpace',   # pg.K_BACKSPACE
            43: 'Tab',         # pg.K_TAB
            44: 'Space',
            45: '-',           # pg.K_MINUS
            46: '=',           # pg.K_EQUALS
            47: '[',           # pg.K_LEFTBRACKET
            48: ']',           # pg.K_RIGHTBRACKET
            49: '\ ',          # pg.K_BACKSLASH

            51: ';',           # pg.K_SEMICOLON
            52: " ' ",         # pg.K_QUOTE
            53: '~',           # pg.K_BACKQUOTE
            54: '< ,',         # pg.K_COMMA
            55: '> .',
            56: '? /',         # pg.K_SLASH
            57: 'Caps Lock',   # pg.K_CAPSLOCK
            58: 'F1',          # pg.K_F1
            59: 'F2',          # pg.K_F2
            60: 'F3',          # pg.K_F3
            61: 'F4',          # pg.K_F4
            62: 'F5',          # pg.K_F5
            63: 'F6',          # pg.K_F6
            64: 'F7',          # pg.K_F7
            65: 'F8',          # pg.K_F8
            66: 'F9',          # pg.K_F9
            67: 'F10',         # pg.K_F10
            68: 'F11',         # pg.K_F11
            69: 'F12',         # pg.K_F12
            70: 'PrintScrean', # pg.K_PRINTSCREEN
            71: 'ScrollLock',  # pg.K_SCROLLLOCK
            72: 'Pause/Break', # pg.K_BREAK
            73: 'Insert',      # pg.K_INSERT
            74: 'Home',        # pg.K_HOME
            75: 'Page Up',     # pg.K_PAGEUP
            76: 'DELETE',      # pg.K_DELETE
            77: 'END',         # pg.K_END
            78: 'Page Down',   # pg.K_PAGEDOWN
            79: 'RIGHT',       # pg.K_RIGHT
            80: 'LEFT',        # pg.K_LEFT
            81: 'DOWN',        # pg.K_DOWN
            82: 'UP',          # pg.K_UP
            83: 'NumLock',     # pg.K_NUMLOCKCLEAR
            84: '/',           # pg.K_KP_DIVIDE
            85: '*',           # pg.K_KP_MULTIPLY
            86: '-',           # pg.K_KP_MINUS
            87: '+',           # pg.K_KP_PLUS
            88: 'Enter Right', # pg.K_KP_ENTER
            89: 'R 1',         # pg.K_KP_1
            90: 'R 2',         # pg.K_KP_2
            91: 'R 3',         # pg.K_KP_3
            92: 'R 4',         # pg.K_KP_4
            93: 'R 5',         # pg.K_KP_5
            94: 'R 6',         # pg.K_KP_6
            95: 'R 7',         # pg.K_KP_7
            96: 'R 8',         # pg.K_KP_8
            97: 'R 9',         # pg.K_KP_9
            98: 'R 0',         # pg.K_KP_0
            99: ' . ',         # pg.K_KP_PERIOD
            100: ' \ ',        # pg.K_BACKSLASH

            224: 'CTRL_L',     # pg.K_LCTRL
            225: 'SHIFT_L',    # pg.K_LSHIFT
            226: 'ALT_L',      # pg.K_LALT
            228: 'CTRL_R',     # pg.K_RCTRL
            229: 'SHIFT_R',    # pg.K_RSHIFT
            230: 'ALT_R',      # pg.K_RALT
        }
        self.joy_keyboard = {
            # w, a, s, d - up, left, back, down
            # i, j, k, l - triangle, square, cross, circle
            # r, f, y, h - l1, l2, r1, r2

            26: 'w',           # pg.K_w
            4:  'a',           # pg.K_a
            22: 's',           # pg.K_s
            7:  'd',           # pg.K_d

            12: 'i',           # pg.K_i
            13: 'j',           # pg.K_j
            14: 'k',           # pg.K_k
            15: 'l',           # pg.K_l

            21: 'r',           # pg.K_r
            9:  'f',           # pg.K_f
            28: 'y',           # pg.K_y
            11: 'h',           # pg.K_h

        }
        # self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(Vector3, 'direction', 10)
        self.keyboard = self.create_timer(0.01, self.keyboard_timer)

        button_draw('1', 'off')
        button_draw('2', 'off')
        button_draw('3', 'off')
        button_draw('4', 'off')
        button_draw('5', 'off')
        button_draw('6', 'off')
        button_draw('7', 'off')
        button_draw('8', 'off')
        button_draw('9', 'off')
        pg.display.update()

    def keyboard_timer(self):

        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                msg = Vector3()
                # self.get_logger().info(str(event))
                if event.scancode == 89:
                    self.button_name = '1'
                    self.xx = -0.7071
                    self.yy = 0.7071

                elif event.scancode == 90:
                    self.button_name = '2'
                    self.xx = -1.0
                    self.yy = 0.0

                elif event.scancode == 91:
                    self.button_name = '3'
                    self.xx = -0.7071
                    self.yy = -0.7071

                elif event.scancode == 92:
                    self.button_name = '4'
                    self.xx = 0.0
                    self.yy = 1.0

                elif event.scancode == 93:
                    self.button_name = '5'
                    self.xx = 0.0
                    self.yy = 0.0

                elif event.scancode == 94:
                    self.button_name = '6'
                    self.xx = 0.0
                    self.yy = -1.0

                elif event.scancode == 95:
                    self.button_name = '7'
                    self.xx = 0.7071
                    self.yy = 0.7071
                    # button_draw(self.button_name, 'on')

                elif event.scancode == 96:
                    self.button_name = '8'
                    self.xx = 1.0
                    self.yy = 0.0

                    # button_draw(self.button_name, 'on')
                    
                    # pg.draw.rect(screen, color_on, masb1, TH1)
                    # img = font.render(self.button_name, True, color_on)
                    # mm = location.get(self.button_name)
                    # # screen.blit(img, (masb1[0] + 10, masb1[1] + 20))
                    # screen.blit(img, (mm[0] + 10, mm[1] + 20))

                elif event.scancode == 97:
                    self.button_name = '9'
                    self.xx = 0.7071
                    self.yy = -0.7071

                if len(self.button_name):
                    button_draw(self.button_name, 'on')
                    # self.get_logger().info(self.button_name)
                    # self.get_logger().info(f'x={self.xx}, y={self.yy}')
                    msg.x = self.xx * self.vel
                    msg.y = self.yy * self.vel
                    self.get_logger().info(f'x={msg.x}, y={msg.y}')
                    self.pub.publish(msg)
                    self.button_name = ''
                    self.xx = 0.0
                    self.yy = 0.0
            
            if event.type == pg.KEYUP:
                button_draw('1', 'off')
                button_draw('2', 'off')
                button_draw('3', 'off')
                button_draw('4', 'off')
                button_draw('5', 'off')
                button_draw('6', 'off')
                button_draw('7', 'off')
                button_draw('8', 'off')
                button_draw('9', 'off')
                # pg.draw.rect(screen, color_off, self.masb1, self.TH1)
                # img = font.render("b1", True, color_off)
                # screen.blit(img, (self.masb1[0] + 10, self.masb1[1] + 20))


        pg.display.update()

    def joy_cb(self, msg: Joy):
        val_header = msg.header
        val_axes = msg.axes
        val_buttons = msg.buttons

        for i in range(len(val_buttons)):
            if val_buttons[i] == 1:
                self.get_logger().info(f'buttons: {self.joy_green_button.get(i)}')

        for i in range(len(val_axes)):
            if val_axes[i] != 0.0:
                key = ''
                if val_axes[i] < 0:
                    key = '-'
                self.get_logger().info(f'axes: {self.joy_green_axes.get(key + str(i))}, val: {round(val_axes[i], 8)}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyGreen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()