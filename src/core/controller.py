import math

import pybullet as p
import random
from core.utils import steering_to_target, track_held_keys, is_facing_target

class Controller:
    def __init__(self):
        self.keys_held = set()

    def generate_random_action(self):
        action = []

        action_move = random.randint(-10, 10) # forward, backward, or no movement
        action_steer = random.randint(-60, 60)/100  # left, right, or no steering
        action_hold = random.randint(1, 480)  # hold the current action or not

        action.append(action_move) # forward or backward
        action.append(action_steer)  # left or right
        action.append(action_hold)

        return action


    def approach_target(self, state, target_pos):
        # Check if we are at the target
        print(math.hypot(state["position"][0] - target_pos[0], state["position"][1] - target_pos[1]))
        if math.hypot(state["position"][0] - target_pos[0], state["position"][1] - target_pos[1]) < 0.1:
            action = [0, 0, 0]
            return action
        steer = steering_to_target(state["position"][0], state["position"][1], state["orientation"], target_pos[0], target_pos[1])
        action = [10, steer, 1]
        return action
        
    def control(self, action, mode):# first position for drive second for steer third for hold
        speed = action[0]
        steer = action[1]
        hold = action[2]

        if mode == "manual":
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

        if mode == "random":
            pass
        
        elif mode == "approach_target":
            pass

        return steer, speed, hold
