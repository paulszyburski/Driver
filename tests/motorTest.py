from lib.mBot import *
from time import sleep

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