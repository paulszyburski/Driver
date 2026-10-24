import sys
from pathlib import Path
from time import sleep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core.hardware.MBotlib.mBot import mBot


def onLight(value):
    print("light =", value)


if __name__ == "__main__":
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")

    while True:
        bot.requestLightOnBoard(1, onLight)
        sleep(0.5)
