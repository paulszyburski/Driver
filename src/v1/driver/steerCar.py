import sys
from pathlib import Path
from threading import Lock
from time import sleep

from pynput import keyboard

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from src.core.hardware.MBotlib.mBot import mBot


FORWARD_POWER = 170
TURN_POWER = 130
LOOP_DELAY_S = 0.02

pressed_keys = set()
pressed_keys_lock = Lock()


def normalize_key(key):
    if hasattr(key, "char") and key.char:
        return key.char.lower()
    if key == keyboard.Key.space:
        return "space"
    return None


def on_press(key):
    normalized = normalize_key(key)
    if normalized is None:
        return

    with pressed_keys_lock:
        pressed_keys.add(normalized)


def on_release(key):
    normalized = normalize_key(key)
    if normalized is None:
        return

    with pressed_keys_lock:
        pressed_keys.discard(normalized)


def get_pressed_keys():
    with pressed_keys_lock:
        return list(pressed_keys)


def main():
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")
    sleep(2)
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while True:
            keys = get_pressed_keys()
            wheels = [0, 0]

            if "w" in keys:
                wheels[0] += FORWARD_POWER
                wheels[1] += FORWARD_POWER
            if "s" in keys:
                wheels[0] -= FORWARD_POWER
                wheels[1] -= FORWARD_POWER
            if "d" in keys:
                wheels[0] += TURN_POWER
            if "a" in keys:
                wheels[1] += TURN_POWER

            if "space" in keys:
                wheels = [0, 0]

            bot.doMove(wheels[0], wheels[1])

            if "q" in keys:
                break

            sleep(LOOP_DELAY_S)
    finally:
        listener.stop()
        bot.doMove(0, 0)
        bot.close()


if __name__ == "__main__":
    main()
