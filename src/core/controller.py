import math
import random
import pybullet as p
from core.utils import approach_target, track_held_keys, adjust_yaw_in_place

class Controller:
    def __init__(self):
        self.keys_held = set()

    def generate_random_action(self):
        action_move = random.randint(-10, 10)  # forward, backward, or no movement
        action_steer = random.uniform(-0.6, 0.6)  # left, right, or no steering
        action_hold = random.randint(1, 480)  # hold duration
        return [action_move, action_steer, action_hold]

    def scripted_controller(self, state, phase):
        yaw_target = -math.pi / 2
        yaw_err = state["orientation"] - yaw_target

        if phase == 0:
            y_pos = state["position"][1]
            target_pos = (0, 0.8) if y_pos >= 0 else (0, -0.8)
            action = approach_target(state, target_pos)
            if action == [0, 0, 0]:
                phase = 1.0
            print(f"Phase 0 action: {action}")
            return action, phase

        if phase in [1.0, 1.1]:
            action, phase = adjust_yaw_in_place(yaw_err, phase)
            if action == [0, 0, 0]:
                phase = 2
            return action, phase

        if phase == 2:
            action = approach_target(state, (0, 0))
            if action == [0, 0, 0]:
                phase = 3
            return action, phase
        
        return [0, 0, 0], phase

    def control(self, action, mode="manual"):
        """
        Processes an action based on the mode.
        Action format: [speed, steer, hold]
        """
        speed, steer, hold = action if action else (0, 0, 1)

        if mode == "manual":
            events = p.getKeyboardEvents()
            track_held_keys(events, self.keys_held)

            steer, speed, hold = 0, 0, 1
            if p.B3G_UP_ARROW in self.keys_held:
                speed = 20
            if p.B3G_DOWN_ARROW in self.keys_held:
                speed = -6
            if p.B3G_LEFT_ARROW in self.keys_held:
                steer = 0.4
            if p.B3G_RIGHT_ARROW in self.keys_held:
                steer = -0.4

        return steer, speed, hold
