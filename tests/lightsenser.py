from lib.mBot import *
from time import sleep

def onLight(value):
    print("light =", value)

if __name__ == "__main__":
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")

    while True:
        bot.requestLightOnBoard(1, onLight)
        sleep(0.5)