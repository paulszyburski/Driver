import pybullet as p
import pybullet_data
import time


def find_joints(car):
    steering_joints = []
    drive_joints = []
    for i in range(p.getNumJoints(car)):
        name = p.getJointInfo(car, i)[1].decode("utf-8")

        if "steering" in name:
            steering_joints.append(i)
        if "wheel" in name and "steering" not in name:
            drive_joints.append(i)  
    return steering_joints, drive_joints

def track_held_keys(events, keys_held):
    for k, v in events.items():
        if v & p.KEY_WAS_TRIGGERED:
            keys_held.add(k)
        if v & p.KEY_WAS_RELEASED:
            keys_held.discard(k)
