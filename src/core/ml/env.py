import math
from random import random

import pybullet as p
import pybullet_data

from core.ml.utils import (
    find_joints,
    dist_to_target,
    dist_to_obstacle,
    random_spawn,
    get_pos,
    get_orientation,
    corner_state,
    get_velocity,
    get_steer_angle,
    check_collision,
)


class Env:
    def __init__(self, connection_mode=p.GUI):
        self.client_id = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        yaw_90 = p.getQuaternionFromEuler([0, 0, math.pi / 2])
        yaw_random = p.getQuaternionFromEuler([0, 0, random() * 2 * math.pi])

        self.plane = p.loadURDF("plane.urdf")
        self.car = p.loadURDF("racecar/racecar.urdf", random_spawn(), yaw_random)
        self.obstacles = {
            "obstacle1": p.loadURDF("racecar/racecar.urdf", [0.35, 0, 0.2], yaw_90),
            "obstacle2": p.loadURDF("racecar/racecar.urdf", [-0.35, 0, 0.2], yaw_90),
        }
        self.steering_joints, self.drive_joints = find_joints(self.car)

        p.setRealTimeSimulation(0)

    def reset(self):
        yaw_90 = p.getQuaternionFromEuler([0, 0, math.pi / 2])
        yaw_random = p.getQuaternionFromEuler([0, 0, random() * 2 * math.pi])

        car_pos = random_spawn()
        p.resetBasePositionAndOrientation(
            self.car, car_pos, yaw_random, physicsClientId=self.client_id
        )
        p.resetBaseVelocity(
            self.car, [0, 0, 0], [0, 0, 0], physicsClientId=self.client_id
        )
        for joint in self.steering_joints + self.drive_joints:
            p.resetJointState(self.car, joint, 0, physicsClientId=self.client_id)

        p.resetBasePositionAndOrientation(
            self.obstacles["obstacle1"],
            [0.35, 0, 0.2],
            yaw_90,
            physicsClientId=self.client_id,
        )
        p.resetBaseVelocity(
            self.obstacles["obstacle1"],
            [0, 0, 0],
            [0, 0, 0],
            physicsClientId=self.client_id,
        )

        p.resetBasePositionAndOrientation(
            self.obstacles["obstacle2"],
            [-0.35, 0, 0.2],
            yaw_90,
            physicsClientId=self.client_id,
        )
        p.resetBaseVelocity(
            self.obstacles["obstacle2"],
            [0, 0, 0],
            [0, 0, 0],
            physicsClientId=self.client_id,
        )

    def follow_camera(self, object, distance=2, yaw=45, pitch=-30):
        pos = get_pos(object, self.client_id)
        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=pos,
        )

    def get_state(self, object):
        pos = get_pos(object, self.client_id)
        obs1_pos = get_pos(self.obstacles["obstacle1"], self.client_id)
        obs2_pos = get_pos(self.obstacles["obstacle2"], self.client_id)
        orientation = get_orientation(object, self.client_id)[2]
        velocity = get_velocity(object, self.client_id)
        steer_angle = get_steer_angle(object, self.steering_joints)
        collision = check_collision(object, self.obstacles, self.client_id)
        corners = corner_state(pos, orientation)
        target_dist = dist_to_target(corners, (0, 0.21))
        obs1_dist = dist_to_obstacle(corners, corner_state(obs1_pos, orientation))
        obs2_dist = dist_to_obstacle(corners, corner_state(obs2_pos, orientation))
        yaw_err = orientation - (-math.pi / 2 if pos[1] >= 0 else math.pi / 2)

        return {
            **corners,
            "x": pos[0],
            "y": pos[1],
            "orientation": orientation,
            "velocityx": velocity[0],
            "velocityy": velocity[1],
            "steer_angle": steer_angle,
            "dist_to_target": target_dist,
            "yaw_err": yaw_err,
            "obs1_dist": obs1_dist,
            "obs2_dist": obs2_dist,
            "collision": collision,
            "action_steer": None,
            "action_speed": None,
        }

    def apply_control(self, steer, speed):
        for j in self.steering_joints:
            p.setJointMotorControl2(
                self.car, j, p.POSITION_CONTROL, targetPosition=steer
            )

        for j in self.drive_joints:
            p.setJointMotorControl2(
                self.car, j, p.VELOCITY_CONTROL, targetVelocity=speed, force=100
            )

    def step(self):
        p.stepSimulation()
