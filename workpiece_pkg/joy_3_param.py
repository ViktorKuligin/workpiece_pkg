#!/usr/bin/env python3
import rclpy
import numpy as np

from sensor_msgs.msg import Joy    # ros2 launch teleop_twist_joy teleop-launch.py

from rclpy.node import Node

class JoyParam(Node):
    def __init__(self):
        super().__init__('joy_buttons_axes')
        self.get_logger().warn("joy button axes start")

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
        self.joy_button = {}
        self.joy_axes = {}

        self.declare_parameter('main_dictionary', 'red')
        self.joy_dict = self.get_parameter('main_dictionary').get_parameter_value().string_value

        if self.joy_dict == 'green':
            self.joy_button = self.joy_green_button
            self.joy_axes = self.joy_green_axes
            self.get_logger().info('main joy is: green')
        else:
            self.joy_dict = 'red'
            self.joy_button = self.joy_red_button
            self.joy_axes = self.joy_red_axes
            self.get_logger().info('main joy is: red')

        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

    def joy_cb(self, msg: Joy):
        val_header = msg.header
        val_axes = msg.axes
        val_buttons = msg.buttons

        for i in range(len(val_buttons)):
            if val_buttons[i] == 1:
                self.get_logger().info(f'buttons: {self.joy_button.get(i)}')

        for i in range(len(val_axes)):
            if val_axes[i] != 0.0:
                key = ''
                if self.joy_dict == 'red':
                    if (i == 4) or (i == 5):
                        continue
                if val_axes[i] < 0:
                    key = '-'
                self.get_logger().info(f'axes: {self.joy_axes.get(key + str(i))}, val: {round(val_axes[i], 6)}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyParam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()