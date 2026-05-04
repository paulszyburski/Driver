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

    successful_episodes = 0
    
    for episode in range(1000):
        record = []
        env.reset()

        step = 0
        phase = 0
        start_pos = env.get_state(env.car)["x"], env.get_state(env.car)["y"]
        start_yaw = env.get_state(env.car)["orientation"]

        while True:
            state = env.get_state(env.car)
            action, phase = controller.scripted_controller(state, phase)
            steer, speed, hold = controller.control(action, mode="approach_target")

            for i in range(hold):
                env.apply_control(steer, speed)
                env.step()
                env.follow_camera(env.car)
                step += 1

                curr_state = [episode, step] + [env.get_state(env.car)[key] for key in ["x", "y", "orientation", "velocityx", "velocityy", "steer_angle", "dist_to_target", "yaw_err", "obs1_dist", "obs2_dist"]] + [steer, speed]
                record.append(curr_state)

            if step > 2500 or state["collision"]:
                print(start_pos)
                print("Episode ended due to step limit or collision.")
                break
            if phase == 3:
                successful_episodes += 1
                print("Episode ended successfully. Successful episodes:", successful_episodes, "Current episode:", episode, "Success rate:", successful_episodes/(episode+1))
                break

        if phase == 3:
            with open(f"data/training_data_V1_{date}.csv", "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(record)
        else:
            with open(f"data/failed_episodes_V1_{date}.csv", "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([start_pos[0], start_pos[1], start_yaw])

if __name__ == "__main__":
    main()