import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from core.hardware.MBotlib.mBot import mBot
from time import sleep

bot = mBot()

def onDistance(dist):
    print("distance:", dist)

if __name__ == "__main__":
    bot.startWithSerial("/dev/ttyUSB0")   # use this for USB
    # bot.startWithHID()                  # don't use this unless HID really works

    while True:
        bot.requestUltrasonicSensor(1, 1, onDistance)
        sleep(0.2)