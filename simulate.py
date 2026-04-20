import pybullet as p
from utils import find_joints, track_held_keys
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.2])
car2 = p.loadURDF("racecar/racecar.urdf", [0.5, 0, 0.2])
car3 = p.loadURDF("racecar/racecar.urdf", [-0.5, 0, 0.2])

steering_joints, drive_joints = find_joints(car)
keys_held = set()

p.setRealTimeSimulation(0)

def control(action: list[int] = []):
    #TODO: use action to control the car instead of the keyboard inputs
    steer = 0
    speed = 0

    if p.B3G_UP_ARROW in keys_held:
        speed = 10
    if p.B3G_DOWN_ARROW in keys_held:
        speed = -6
    if p.B3G_LEFT_ARROW in keys_held:
        steer = 0.4
    if p.B3G_RIGHT_ARROW in keys_held:
        steer = -0.4

    for j in steering_joints:
        p.setJointMotorControl2(
            car,
            j,
            p.POSITION_CONTROL,
            targetPosition=steer
        )

    for j in drive_joints:
        p.setJointMotorControl2(
            car,
            j,
            p.VELOCITY_CONTROL,
            targetVelocity=speed,
            force=100
        )
    return steer, speed
    
def main():
    step = 0
    while True:
        events = p.getKeyboardEvents()

        track_held_keys(events, keys_held)
        control()

        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()    
