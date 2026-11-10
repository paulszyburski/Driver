import sys
from pathlib import Path
from time import sleep

import cv2
import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1]))

from core.hardware.camera import Camera
from core.hardware.MBotlib.mBot import mBot


def normalize_angle(angle):
    """Convert an angle to the range -180 to 180."""
    return (angle + 180) % 360 - 180


def main():
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")

    sleep(2)

    camera = Camera((640, 480), np.array([5, 180, 120]), np.array([28, 255, 255]))

    camera.configure_camera()

    picture = camera.take_picture()

    if picture is None:
        raise RuntimeError("The camera did not return an image.")

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
