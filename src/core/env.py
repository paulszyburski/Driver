import pybullet as p
import pybullet_data
import time

from utils import find_joints, track_held_keys, get_pos, get_orientation, compute_power, compute_corners_position, get_velocity, get_steer_angle, check_collision


class Env:
    def __init__(self):
        self.client_id = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.plane = p.loadURDF("plane.urdf")
        self.car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0])
        self.obstacles = {"obstacle1": p.loadURDF("racecar/racecar.urdf", [0.5, 0, 0.2]), "obstacle2": p.loadURDF("racecar/racecar.urdf", [-0.5, 0, 0.2])}

        self.steering_joints, self.drive_joints = find_joints(self.car)
        self.keys_held = set()

        p.setRealTimeSimulation(0)
        p.resetDebugVisualizerCamera(
            cameraDistance=2,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.2],
        )

    def get_state(self):
        pos = get_pos(self.car, self.client_id)
        orientation = get_orientation(self.car, self.client_id)[2]
        velocity = get_velocity(self.car, self.client_id)
        steer_angle = get_steer_angle(self.car, self.steering_joints)
        collision = check_collision(self.car, self.obstacles, self.client_id)
        front_middle, front_right, front_left, back_middle, back_right, back_left = compute_corners_position(pos, orientation)
        return {
            "position": pos,
            "orientation": orientation,
            "velocity": velocity,
            "steer_angle": steer_angle,
            "collision": collision,
            "front_middle": front_middle,
            "front_right": front_right,
            "front_left": front_left,
            "back_middle": back_middle,
            "back_right": back_right,
            "back_left": back_left
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