import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from time import sleep

from core.hardware.MBotlib.mBot import mBot

bot = mBot()


def onDistance(dist):
    print("distance:", dist)


if __name__ == "__main__":
    bot.startWithSerial("/dev/ttyUSB0")
    # bot.startWithHID()                  # don't use this unless HID really works

    while True:
        bot.requestUltrasonicSensor(1, 1, onDistance)
        sleep(0.2)
