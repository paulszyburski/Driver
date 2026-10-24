import sys
from pathlib import Path
from time import sleep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core.hardware.MBotlib.mBot import mBot


def onLineFollower(value):
    print("status =", value)


if __name__ == "__main__":
    bot = mBot()

    bot.startWithHID()
    while 1:
        bot.requestLineFollower(1, 2, onLineFollower)
        sleep(0.5)
