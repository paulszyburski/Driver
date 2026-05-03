import sys
from pathlib import Path
# Add the project root to sys.path if not already there
PROJECT_ROOT = str(Path(__file__).resolve().parent.parent.parent)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import pybullet as p
import time
from core.env import Env
from core.controller import Controller

def main():
    env = Env()
    controller = Controller()

    while True:
        state = env.get_state(env.car)
        # In demo mode, we use manual control
        steer, speed, _ = controller.control(None, mode="manual")
        env.apply_control(steer, speed)
        env.step()
        env.follow_camera(env.car)
        
        # Display state periodically
        if int(time.time() * 10) % 50 == 0:  # Roughly every 5 seconds
             print(f"Pos: {state['position']}, Collision: {state['collision']}")

        time.sleep(1 / 240)

if __name__ == "__main__":
    main()
