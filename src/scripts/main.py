import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.ml.env import Env
from core.ml.utils import compute_power
from core.ml.controller import Controller
from core.ml.utils import log
import time


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