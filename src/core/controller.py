import pybullet as p
import random
from core.utils import track_held_keys

class Controller:
    def __init__(self):
        self.keys_held = set()

    def generate_random_action(self):
        action = []

        action_move = random.choice([1, 2, 0])  # forward, backward, or no movement
        action_steer = random.choice([3, 4, 0])  # left, right, or no steering
        action_hold = random.randint(1, 480)  # hold the current action or not

        action.append(action_move) # forward or backward
        action.append(action_steer)  # left or right
        action.append(action_hold)

        return action

    def control(self, action: list[int] = []):# [1, 2, 3, 4] for forward, backward, left, right
        #events = p.getKeyboardEvents()
        #track_held_keys(events, self.keys_held)

        #steer, speed = 0, 0
        #if p.B3G_UP_ARROW in self.keys_held:
        #    speed = 20
        #if p.B3G_DOWN_ARROW in self.keys_held:
        #    speed = -6
        #if p.B3G_LEFT_ARROW in self.keys_held:
        #    steer = 0.4
        #if p.B3G_RIGHT_ARROW in self.keys_held:
        #    steer = -0.4

        steer, speed, hold = 0, 0, 0
        print(action)

        if 1 in action:  # forward
            speed = 20
        elif 2 in action: # backward
            speed = -6
        if 3 in action: # left
            steer = 0.4
        elif 4 in action: # right
            steer = -0.4
        hold = action[2] 

        return steer, speed, hold