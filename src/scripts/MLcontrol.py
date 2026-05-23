
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.env import Env
from core.utils import compute_power
from core.controller import Controller
from core.utils import log
from useml import predict
import time


def main():
    env = Env()
    controller = Controller()

    step = 0
    phase = 0

    while True:
        state = env.get_state(env.car)
        steer, speed = predict(*list(state.values())[12:22])

        env.apply_control(steer, speed)
        env.step()
        env.follow_camera(env.car)
        time.sleep(1 / 240)

        log(state, step)
        step += 1

if __name__ == "__main__":
    main()