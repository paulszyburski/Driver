from lib.mBot import *
from time import sleep

bot = mBot()

def onDistance(dist):
    print("distance:", dist)

if __name__ == "__main__":
    bot.startWithSerial("/dev/ttyUSB0")   # use this for USB
    # bot.startWithHID()                  # don't use this unless HID really works

    while True:
        bot.requestUltrasonicSensor(1, 4, onDistance)
        sleep(0.2)