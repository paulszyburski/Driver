import random

import pybullet as p
import pybullet_data
import time
import math


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

def get_pos(object, client_id):
    position = p.getBasePositionAndOrientation(object, physicsClientId=client_id)[0]
    return position

def get_orientation(object, client_id):
    roll, pitch, yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(object, physicsClientId=client_id)[1])
    return roll, pitch, yaw

def get_velocity(object, client_id):
    velocity = p.getBaseVelocity(object, physicsClientId=client_id)[0]
    return velocity

def get_steer_angle(car, steering_joints):
    steer_angle = p.getJointState(car, steering_joints[0])[0]
    return steer_angle

def compute_power(value: str):
    from decimal import Decimal

    d = Decimal(value)
    return format(d, "f")

def check_collision(object, obstacles, client_id):

    for obstacle in obstacles.values():
        contact_points = p.getContactPoints(object, obstacle, physicsClientId=client_id)
        if len(contact_points) > 0:
            return True
    return False

def random_spawn(x_range=(-5, 5), y_range=(-5, 5)):
    x = random.randint(x_range[0]*100, x_range[1]*100)/100
    y = random.randint(y_range[0]*100, y_range[1]*100)/100
    return x, y, 0

def log(state, step):
    if step % 550 != 0:
        return
    print(f"X: {state['position'][0]:.2f}, Y: {state['position'][1]:.2f}, Z: {state['position'][2]:.2f}")
    print(f"Collision: {state['collision']}")
    print(f"Orientation yaw: {compute_power(str(state['orientation']))}")
    print(f"Steer angle: {compute_power(str(state['steer_angle']))}")
    print(f"Velocity: {compute_power(str(state['velocity'][0]))}, {compute_power(str(state['velocity'][1]))}, {compute_power(str(state['velocity'][2]))}")
    print(f"Front middle: {compute_power(str(state['front_middle'][0]))}, {compute_power(str(state['front_middle'][1]))}")
    print(f"Front right: {compute_power(str(state['front_right'][0]))}, {compute_power(str(state['front_right'][1]))}")
    print(f"Front left: {compute_power(str(state['front_left'][0]))}, {compute_power(str(state['front_left'][1]))}")
    print(f"Back middle: {compute_power(str(state['back_middle'][0]))}, {compute_power(str(state['back_middle'][1]))}")
    print(f"Back right: {compute_power(str(state['back_right'][0]))}, {compute_power(str(state['back_right'][1]))}")
    print(f"Back left: {compute_power(str(state['back_left'][0]))}, {compute_power(str(state['back_left'][1]))}\n\n")

def is_facing_target(x0, y0, orientation, x1, y1, eps=0.6):
    dx = x1 - x0
    dy = y1 - y0

    if abs(dx * math.sin(orientation) - dy * math.cos(orientation)) > eps:
        return False
    
    t = dx * math.cos(orientation) + dy * math.sin(orientation)
    return True if t >= 0 else False

def steering_to_target(x0, y0, orientation, x1, y1, max_steer=0.6):
    dx = x1 - x0
    dy = y1 - y0

    target_angle = math.atan2(dy, dx)
    
    delta = target_angle - orientation
    delta = math.atan2(math.sin(delta), math.cos(delta))
    
    k = 1.0
    steering = max(-max_steer, min(max_steer, k * delta))
    
    return steering

def adjust_yaw(orientation, target_yaw, max_delta=0.1):
    delta = target_yaw - orientation
    delta = math.atan2(math.sin(delta), math.cos(delta))
    
    if abs(delta) < max_delta:
        return target_yaw
    else:
        return orientation + max(-max_delta, min(max_delta, delta))

def compute_corners_position(position, yaw, length=0.43, width=0.29):
    """"
    a1: front middle
    a2: front right
    a3: front left
    a4: back middle
    a5: back right
    a6: back left
    """
    import math
    r1 = length / 2
    a1 = (position[0] + r1 * math.cos(yaw), position[1] + r1 * math.sin(yaw))
    
    r2 = math.sqrt((length / 2) ** 2 + (width / 2) ** 2)
    a2 = (position[0] + math.cos(yaw - math.asin(width / (2 * r2))) * r2, position[1] + math.sin(yaw - math.asin(width / (2 * r2))) * r2)
    
    r3 = math.sqrt((length / 2) ** 2 + (width / 2) ** 2)
    a3 = (position[0] + math.cos(yaw + math.asin(width / (2 * r3))) * r3, position[1] + math.sin(yaw + math.asin(width / (2 * r3))) * r3)
    
    r4 = length / 2
    a4 = (position[0] - r4 * math.cos(yaw), position[1] - r4 * math.sin(yaw))
    
    r5 = math.sqrt((length / 2) ** 2 + (width / 2) ** 2)
    a5 = (position[0] + math.cos(yaw + math.pi - math.asin(width / (2 * r5))) * r5, position[1] + math.sin(yaw + math.pi - math.asin(width / (2 * r5))) * r5)
    
    r6 = math.sqrt((length / 2) ** 2 + (width / 2) ** 2)
    a6 = (position[0] + math.cos(yaw + math.pi + math.asin(width / (2 * r6))) * r6, position[1] + math.sin(yaw + math.pi + math.asin(width / (2 * r6))) * r6)
    
    return a1, a2, a3, a4, a5, a6

