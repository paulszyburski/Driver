import random

import pybullet as p
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
    roll, pitch, yaw = p.getEulerFromQuaternion(
        p.getBasePositionAndOrientation(object, physicsClientId=client_id)[1]
    )
    return roll, pitch, yaw


def get_velocity(object, client_id):
    velocity = p.getBaseVelocity(object, physicsClientId=client_id)[0]
    return velocity


def get_steer_angle(car, steering_joints):
    steer_angle = p.getJointState(car, steering_joints[0])[0]
    return steer_angle


def check_collision(object, obstacles, client_id):
    for obstacle in obstacles.values():
        contact_points = p.getContactPoints(object, obstacle, physicsClientId=client_id)
        if len(contact_points) > 0:
            return True
    return False


def random_spawn(x_range=(-5, 5), y_range=(-5, 5)):
    x = random.randint(x_range[0] * 100, x_range[1] * 100) / 100
    y = random.randint(y_range[0] * 100, y_range[1] * 100) / 100
    return x, y, 0


def log(state, step):
    if step % 550 != 0:
        return
    print(
        f"Position: ({state['x']:.3f}, {state['y']:.3f}) | "
        f"Yaw: {state['orientation']:.3f} | "
        f"Target: {state['dist_to_target']:.3f} | "
        f"Collision: {state['collision']}"
    )


def dist_to_target(ego_state, target_pos):
    return min(
        math.hypot(
            ego_state[name + "x"] - target_pos[0], ego_state[name + "y"] - target_pos[1]
        )
        for name in CORNER_NAMES
    )


def dist_to_obstacle(ego_state, obstacle_state):
    return min(
        math.hypot(
            ego_state[ego + "x"] - obstacle_state[obs + "x"],
            ego_state[ego + "x"] - obstacle_state[obs + "x"],
        )
        for ego in CORNER_NAMES
        for obs in CORNER_NAMES
    )


def steering_to_target(x0, y0, orientation, x1, y1, max_steer=0.6):
    dx = x1 - x0
    dy = y1 - y0

    target_angle = math.atan2(dy, dx)

    delta = target_angle - orientation
    delta = math.atan2(math.sin(delta), math.cos(delta))

    return max(-max_steer, min(max_steer, delta))


def adjust_yaw_in_place(yaw_err, phase):
    direction = 1 if yaw_err > 0 else -1
    if abs(yaw_err) > 0.1:
        if phase == 1.0:
            action = [-7, 0.6 if direction == 1 else -0.6, 80]
            phase = 1.1
        elif phase == 1.1:
            action = [7, -0.6 if direction == 1 else 0.6, 80]
            phase = 1.0
        return action, phase
    else:
        phase = 2
        return [0, 0, 0], phase


def approach_target(state, target_pos):
    if math.hypot(state["x"] - target_pos[0], state["y"] - target_pos[1]) < 0.1:
        return [0, 0, 0]
    steer = steering_to_target(
        state["x"], state["y"], state["orientation"], target_pos[0], target_pos[1]
    )
    return [20, steer, 1]


def compute_corners_position(position, yaw, length=0.43, width=0.29):
    half_length, half_width = length / 2, width / 2
    offsets = (
        (half_length, 0),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, 0),
        (-half_length, half_width),
        (-half_length, -half_width),
    )
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return tuple(
        (
            position[0] + x * cos_yaw - y * sin_yaw,
            position[1] + x * sin_yaw + y * cos_yaw,
        )
        for x, y in offsets
    )


CORNER_NAMES = (
    "front_middle",
    "front_right",
    "front_left",
    "back_middle",
    "back_right",
    "back_left",
)


def corner_state(position, yaw):
    return {
        name + axis: value
        for name, point in zip(CORNER_NAMES, compute_corners_position(position, yaw))
        for axis, value in zip("xy", point)
    }
