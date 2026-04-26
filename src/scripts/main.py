import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.env import Env
from core.utils import compute_power
from core.controller import Controller
import time

def log(state, step):
    if step % 550 != 0:
        return
    print(f"X: {state['position'][0]:.2f}, Y: {state['position'][1]:.2f}, Z: {state['position'][2]:.2f}")
    print(f"Collision: {state['collision']}")
    print(f"Orientation yaw: {compute_power(str(state['orientation']))}")
    print(f"Steer angle: {compute_power(str(state['steer_angle']))}")
    print(f"Velocity: {compute_power(str(state['velocity'][0]))}, {compute_power(str(state['velocity'][1]))}, {compute_power(str(state['velocity'][2]))}")
    print(f"Front middle: {compute_power(str(state['front_middle'][0]))}, {compute_power(str(state['front_middle'][1]))}")
    print(f"Front right: {compute_power(str(state['front_right'][0]))}, {compute_power(str(state['front_right'][1]))}")
    print(f"Front left: {compute_power(str(state['front_left'][0]))}, {compute_power(str(state['front_left'][1]))}")
    print(f"Back middle: {compute_power(str(state['back_middle'][0]))}, {compute_power(str(state['back_middle'][1]))}")
    print(f"Back right: {compute_power(str(state['back_right'][0]))}, {compute_power(str(state['back_right'][1]))}")
    print(f"Back left: {compute_power(str(state['back_left'][0]))}, {compute_power(str(state['back_left'][1]))}\n\n")

def main():
    env = Env()
    controller = Controller()

    step = 0
    while True:
        state = env.get_state(env.car)
        action = controller.generate_random_action()
        steer, speed, hold = controller.control(action)
        for i in range(hold):
            env.apply_control(steer, speed)
            env.step()
            env.follow_camera(env.car)
            time.sleep(1 / 240)

            log(state, step)
            step += 1

if __name__ == "__main__":
    main()