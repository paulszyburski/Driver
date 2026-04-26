import pybullet as p
from utils import track_held_keys

class Controller:
    def __init__(self):
        self.keys_held = set()

    def control(self, action: list[int] = []):
        events = p.getKeyboardEvents()
        track_held_keys(events, self.keys_held)

        steer, speed = 0, 0
        if p.B3G_UP_ARROW in self.keys_held:
            speed = 20
        if p.B3G_DOWN_ARROW in self.keys_held:
            speed = -6
        if p.B3G_LEFT_ARROW in self.keys_held:
            steer = 0.4
        if p.B3G_RIGHT_ARROW in self.keys_held:
            steer = -0.4

        return steer, speed