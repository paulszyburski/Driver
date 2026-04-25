import pybullet as p
from utils import find_joints, track_held_keys, get_pos, get_orientation, compute_power, compute_corners_position, get_velocity, get_steer_angle
import pybullet_data
import time

client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0])
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

        position = get_pos(car, client_id)
        orientation = get_orientation(car, client_id)[2]
        front_middle, front_right, front_left, back_middle, back_right, back_left = compute_corners_position(position, orientation)
        velocity = get_velocity(car, client_id)
        steer_angle = get_steer_angle(car, steering_joints)
        

        if step % 550 == 0:
            print(f"X: {position[0]:.2f}, Y: {position[1]:.2f}, Z: {position[2]:.2f}")
            print(f"Orientation yaw: {compute_power(str(orientation))}")
            print(f"Steer angle: {compute_power(str(steer_angle))}")
            print(f"Velocity: {compute_power(str(velocity[0]))}, {compute_power(str(velocity[1]))}, {compute_power(str(velocity[2]))}")
            print(f"Front middle: {compute_power(str(front_middle[0]))}, {compute_power(str(front_middle[1]))}")
            print(f"Front right: {compute_power(str(front_right[0]))}, {compute_power(str(front_right[1]))}")
            print(f"Front left: {compute_power(str(front_left[0]))}, {compute_power(str(front_left[1]))}")
            print(f"Back middle: {compute_power(str(back_middle[0]))}, {compute_power(str(back_middle[1]))}")
            print(f"Back right: {compute_power(str(back_right[0]))}, {compute_power(str(back_right[1]))}")
            print(f"Back left: {compute_power(str(back_left[0]))}, {compute_power(str(back_left[1]))}\n\n")

        step += 1

        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()    
