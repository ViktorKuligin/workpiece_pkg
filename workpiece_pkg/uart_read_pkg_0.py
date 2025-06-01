import serial
import time
import cv2
import numpy as np

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
USB = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=0)

byte_start = bytes([170])
byte_end = bytes([187])
msg_len_max = 32

while 1:

    data_byte = USB.read()

    if data_byte == byte_start:
        msg = []
        msg.append(int.from_bytes(data_byte,"big"))

        for i in range(msg_len_max):
            data_byte = USB.read()
            if i == msg_len_max-1 and data_byte != byte_end:
                msg = []
                break
            msg.append(int.from_bytes(data_byte,"big"))
            if data_byte == byte_end:
                break

        if len(msg):
            time_now = time.perf_counter()
            time_txt = str(round(time_now,6)).ljust(12,"0")
            print(f'time = {time_txt}, msg = {msg}')
            msg_old = msg
