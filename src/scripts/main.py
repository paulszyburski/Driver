import sys
from pathlib import Path
# Add the project root to sys.path if not already there
PROJECT_ROOT = str(Path(__file__).resolve().parent.parent)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import pybullet as p
import time
from core.env import Env
from core.controller import Controller
from core.utils import log


def main():
    env = Env()
    controller = Controller()

    step = 0
    phase = 0
    while True:
        state = env.get_state(env.car)
        action, phase = controller.scripted_controller(state, phase)
        steer, speed, hold = controller.control(action, mode="approach_target")
        for i in range(hold):
            env.apply_control(steer, speed)
            env.step()
            env.follow_camera(env.car)
            time.sleep(1 / 240)

            log(state, step)
            step += 1

if __name__ == "__main__":
    main()