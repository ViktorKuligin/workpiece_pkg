import serial
import time

PORT = "/dev/ttyACM0"
BAUDRATE = 9600
USB = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=0)

while 1:

    data_byte = USB.read()
    time_now = time.perf_counter()

    if data_byte:

        time_now = time.perf_counter()
        time_txt = str(round(time_now,6)).ljust(12,"0")
        print(f'time = {time_txt}, msg = {data_byte}, from_bytes = {int.from_bytes(data_byte,"big")}')
