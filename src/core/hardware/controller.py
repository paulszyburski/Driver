from time import sleep

from utils import scan_with_ultrasonic
from MBotlib.mBot import mBot


class BotController:
    def __init__(self, bot):
        self.bot = bot

    def calibrate(self):
        scan_with_ultrasonic(self.bot)


if __name__ == "__main__":
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")

    sleep(2)

    controller = BotController(bot)

    try:
        controller.calibrate()
    finally:
        bot.doMove(0, 0)
        bot.close()