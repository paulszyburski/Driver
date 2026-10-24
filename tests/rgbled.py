import sys
from pathlib import Path
from time import sleep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core.hardware.MBotlib.mBot import mBot

if __name__ == "__main__":
    bot = mBot()

    bot.startWithHID()
    while 1:
        bot.doRGBLedOnBoard(1, 100, 0, 0)
        sleep(0.5)
        bot.doRGBLedOnBoard(1, 0, 100, 0)
        sleep(0.5)
