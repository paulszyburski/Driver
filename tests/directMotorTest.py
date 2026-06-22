import serial
import struct
from time import sleep

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=1)

# mCore resets after serial opens
sleep(3)

def move(left_speed, right_speed):
    packet = bytearray([0xff, 0x55, 0x07, 0x00, 0x02, 0x05])
    packet += struct.pack("<h", -left_speed)   # left motor is inverted in this protocol
    packet += struct.pack("<h", right_speed)
    print("sending:", packet.hex(), flush=True)
    ser.write(packet)
    ser.flush()

try:
    print("forward", flush=True)
    move(200, 200)
    sleep(3)

    print("backward", flush=True)
    move(-200, -200)
    sleep(3)

    print("stop", flush=True)
    move(0, 0)
    sleep(1)

finally:
    move(0, 0)
    ser.close()