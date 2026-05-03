import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.env import Env
from core.utils import compute_power
from core.controller import Controller
from core.utils import log
import time
import csv
import datetime

def main():
    date = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    env = Env() 
    controller = Controller()  
    

    for episode in range(10):
        record = []
        env.reset()

        step = 0
        phase = 0
        start_pos = env.get_state(env.car)["x"], env.get_state(env.car)["y"]

        while True:
            state = env.get_state(env.car)
            action, phase = controller.scripted_controller(state, phase)
            steer, speed, hold = controller.control(action, mode="approach_target")

            for i in range(hold):
                env.apply_control(steer, speed)
                env.step()
                env.follow_camera(env.car)
                time.sleep(1 / 240)
                step += 1

                curr_state = [episode, step] + list(env.get_state(env.car).values())
                record.append(curr_state)

            if step > 2500 or state["collision"]:
                print(start_pos)
                print("Episode ended due to step limit or collision.")
                break
            if phase == 3:
                print("Episode ended successfully.")
                break

        if phase == 3:
            with open(f"training_data_V1_{date}.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(record)

if __name__ == "__main__":
    main()