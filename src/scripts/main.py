import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.env import Env
from core.controller import Controller
import time

def log(state, step):
    pass

def main():
    env = Env()
    controller = Controller()

    while True:
        state = env.get_state(env.car)
        action = controller.generate_random_action()
        steer, speed, hold = controller.control(action)
        for i in range(hold):
            env.apply_control(steer, speed)
            env.step()
            env.follow_camera(env.car)
            time.sleep(1 / 240)

if __name__ == "__main__":
    main()