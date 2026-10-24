import sys
from pathlib import Path
from time import sleep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core.hardware.MBotlib.mBot import mBot

bot = mBot()
bot.startWithSerial("/dev/ttyUSB0")

sleep(2)

try:
    print("forward")
    bot.doMove(120, 80)
    sleep(3)

    print("stop")
    bot.doMove(0, 0)
    sleep(2)

except KeyboardInterrupt:
    bot.doMove(0, 0)
