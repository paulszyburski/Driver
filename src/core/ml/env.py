import math
from random import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pybullet as p
import pybullet_data
import time

from core.ML.utils import find_joints, dist_to_target, dist_to_obstacle, random_spawn, track_held_keys, get_pos, get_orientation, compute_power, compute_corners_position, get_velocity, get_steer_angle, check_collision


class Env:
    def __init__(self):
        self.client_id = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        yaw_90 = p.getQuaternionFromEuler([0, 0, math.pi/2])
        yaw_random = p.getQuaternionFromEuler([0, 0, random()*2*math.pi])

        self.plane = p.loadURDF("plane.urdf")
        self.car = p.loadURDF("racecar/racecar.urdf", random_spawn(), yaw_random)
        self.obstacles = {
          "obstacle1": p.loadURDF("racecar/racecar.urdf", [0.35, 0, 0.2], yaw_90),
          "obstacle2": p.loadURDF("racecar/racecar.urdf", [-0.35, 0, 0.2], yaw_90)
        }
        self.steering_joints, self.drive_joints = find_joints(self.car)
        self.keys_held = set()

        p.setRealTimeSimulation(0)

    def reset(self):
        yaw_90 = p.getQuaternionFromEuler([0, 0, math.pi/2])
        yaw_random = p.getQuaternionFromEuler([0, 0, random()*2*math.pi])
        
        car_pos = random_spawn()
        p.resetBasePositionAndOrientation(self.car, car_pos, yaw_random, physicsClientId=self.client_id)
        p.resetBaseVelocity(self.car, [0, 0, 0], [0, 0, 0], physicsClientId=self.client_id)
        for joint in self.steering_joints + self.drive_joints:
            p.resetJointState(self.car, joint, 0, physicsClientId=self.client_id)
        
        p.resetBasePositionAndOrientation(self.obstacles["obstacle1"], [0.35, 0, 0.2], yaw_90, physicsClientId=self.client_id)
        p.resetBaseVelocity(self.obstacles["obstacle1"], [0, 0, 0], [0, 0, 0], physicsClientId=self.client_id)
        
        p.resetBasePositionAndOrientation(self.obstacles["obstacle2"], [-0.35, 0, 0.2], yaw_90, physicsClientId=self.client_id)
        p.resetBaseVelocity(self.obstacles["obstacle2"], [0, 0, 0], [0, 0, 0], physicsClientId=self.client_id)

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
        front_middle, front_right, front_left, back_middle, back_right, back_left = compute_corners_position(pos, orientation)
        obs1_fm, obs1_fr, obs1_fl, obs1_bm, obs1_br, obs1_bl = compute_corners_position(obs1_pos, orientation)
        obs2_fm, obs2_fr, obs2_fl, obs2_bm, obs2_br, obs2_bl = compute_corners_position(obs2_pos, orientation)
        target_dist = dist_to_target({"front_middlex": front_middle[0], "front_middley": front_middle[1], "front_rightx": front_right[0], "front_righty": front_right[1], "front_leftx": front_left[0], "front_lefty": front_left[1], "back_middlex": back_middle[0], "back_middley": back_middle[1], "back_rightx": back_right[0], "back_righty": back_right[1], "back_leftx": back_left[0], "back_lefty": back_left[1]}, (0, 0.21))
        obs1_dist = dist_to_obstacle({"front_middlex": front_middle[0], "front_middley": front_middle[1], "front_rightx": front_right[0], "front_righty": front_right[1], "front_leftx": front_left[0], "front_lefty": front_left[1], "back_middlex": back_middle[0], "back_middley": back_middle[1], "back_rightx": back_right[0], "back_righty": back_right[1], "back_leftx": back_left[0], "back_lefty": back_left[1]}, {"front_middlex": obs1_fm[0], "front_middley": obs1_fm[1], "front_rightx": obs1_fr[0], "front_righty": obs1_fr[1], "front_leftx": obs1_fl[0], "front_lefty": obs1_fl[1], "back_middlex": obs1_bm[0], "back_middley": obs1_bm[1], "back_rightx": obs1_br[0], "back_righty": obs1_br[1], "back_leftx": obs1_bl[0], "back_lefty": obs1_bl[1]})
        obs2_dist = dist_to_obstacle({"front_middlex": front_middle[0], "front_middley": front_middle[1], "front_rightx": front_right[0], "front_righty": front_right[1], "front_leftx": front_left[0], "front_lefty": front_left[1], "back_middlex": back_middle[0], "back_middley": back_middle[1], "back_rightx": back_right[0], "back_righty": back_right[1], "back_leftx": back_left[0], "back_lefty": back_left[1]}, {"front_middlex": obs2_fm[0], "front_middley": obs2_fm[1], "front_rightx": obs2_fr[0], "front_righty": obs2_fr[1], "front_leftx": obs2_fl[0], "front_lefty": obs2_fl[1], "back_middlex": obs2_bm[0], "back_middley": obs2_bm[1], "back_rightx": obs2_br[0], "back_righty": obs2_br[1], "back_leftx": obs2_bl[0], "back_lefty": obs2_bl[1]})
        yaw_err = orientation - (-math.pi/2 if pos[1] >= 0 else math.pi/2)

        return {
            "front_middlex": front_middle[0],
            "front_middley": front_middle[1],
            "front_rightx": front_right[0],
            "front_righty": front_right[1],
            "front_leftx": front_left[0],
            "front_lefty": front_left[1],
            "back_middlex": back_middle[0],
            "back_middley": back_middle[1],
            "back_rightx": back_right[0],
            "back_righty": back_right[1],
            "back_leftx": back_left[0],
            "back_lefty": back_left[1],
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
    
    def step(self, action: list[int] = []):
        p.stepSimulation()