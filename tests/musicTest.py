import sys
from pathlib import Path
from time import sleep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core.hardware.MBotlib.mBot import mBot
from multiprocessing import freeze_support

if __name__ == "__main__":
    freeze_support()
    bot = mBot()

    bot.startWithHID()
    tones = {"C4": 262, "D4": 294, "E4": 330, "F4": 349, "G4": 392}
    music = "E4 E4 F4 G4 G4 F4 E4 D4 C4 C4 D4 E4 E4 D4 D4".split()
    while 1:
        try:
            for note in music:
                bot.doBuzzer(tones[note], 500)
                sleep(0.5)
        except KeyboardInterrupt:
            bot.doBuzzer(0)
        bot.doBuzzer(0)
        sleep(3)
