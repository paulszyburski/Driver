from env import Env
from controller import Controller
import time

def log(state, step):
    pass

def main():
    env = Env()
    controller = Controller()

    while True:
        state = env.get_state()
        steer, speed = controller.control()
        env.apply_control(steer, speed)
        env.step()
        time.sleep(1 / 240)

if __name__ == "__main__":
    main()