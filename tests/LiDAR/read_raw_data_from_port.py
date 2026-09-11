import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Reading LiDAR...")
while True:
    data = ser.read(128)
    if data:
        print(data.hex(" "))
    else:
        print("no data")
    time.sleep(0.2)