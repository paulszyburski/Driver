"""
Parking data generator for autopark ML training.

Scene layout (top-down, x-axis):
  car3 [-0.5, 0]  |  ego [0, 0]  |  car2 [0.5, 0]

Generates episodes where the ego car starts at a random position on the
plane and uses a scripted rule-based controller to park in the centre spot.
Each timestep's observation and the action taken are recorded.

Run:  python generate_data.py
Output: data/parking_data.csv
"""

import csv
import math
import os
import random

import pybullet as p
import pybullet_data

from utils import find_joints

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
TARGET_POS = [0.0, 0.0]  # x, y of the parking spot
# Car approaches from +y and parks pointing south (-y direction)
TARGET_YAW = -math.pi / 2  # desired heading when parked (radians)
# Approach waypoint: north of the spot, clear of both parked cars
APPROACH_WP = [0.0, 1.8]

PARKED_CAR2_POS = [0.5, 0.0]
PARKED_CAR3_POS = [-0.5, 0.0]

TARGET_SUCCESS_EPISODES = 1000  # how many successful episodes to collect
MAX_ATTEMPTS = 12000  # give up after this many total attempts

MAX_STEPS = 2500  # max control steps per episode
PHYSICS_SUB_STEPS = 8  # physics steps per control step
ACTION_NOISE_STEER = 0.03  # Gaussian noise on steering
ACTION_NOISE_SPEED = 0.3  # Gaussian noise on speed

SUCCESS_DIST = 0.28  # metres  – must be within this of target
SUCCESS_YAW_ERR = 0.8  # radians – must be within this of target yaw
SUCCESS_SPEED = 2.0  # m/s     – must be slower than this

RANDOM_START_MIN_DIST = 1.5  # min distance from target for random start
RANDOM_START_BOUND = 5.0  # ± metres for random start x/y

OUTPUT_DIR = "data"
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "parking_data.csv")

CSV_COLUMNS = [
    "episode",
    "step",
    # --- ego state ---
    "x",
    "y",
    "yaw",
    "vx",
    "vy",
    "omega",
    # --- relative to target ---
    "dist_to_target",
    "angle_to_target",
    "heading_error",
    "yaw_error",
    # --- relative positions of parked cars ---
    "obs2_dx",
    "obs2_dy",
    "obs3_dx",
    "obs3_dy",
    # --- actions applied at this step ---
    "action_steer",
    "action_speed",
    # --- episode outcome (set after episode ends) ---
    "success",
]


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def get_state(body_id):
    """Return (pos_3d, yaw, vel_3d, ang_vel_z)."""
    pos, orn = p.getBasePositionAndOrientation(body_id)
    vel, ang_vel = p.getBaseVelocity(body_id)
    yaw = p.getEulerFromQuaternion(orn)[2]
    return pos, yaw, vel, ang_vel[2]


def reset_car(body_id, xy, yaw):
    """Teleport a car to xy position with given yaw, zero velocity."""
    orn = p.getQuaternionFromEuler([0.0, 0.0, yaw])
    p.resetBasePositionAndOrientation(body_id, [xy[0], xy[1], 0.2], orn)
    p.resetBaseVelocity(body_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    # Reset all joint states so wheels don't carry spin from previous episode
    num_joints = p.getNumJoints(body_id)
    for j in range(num_joints):
        p.resetJointState(body_id, j, 0.0, 0.0)


def random_start():
    """Sample a random (xy, yaw) that is safely outside the parking area."""
    while True:
        x = random.uniform(-RANDOM_START_BOUND, RANDOM_START_BOUND)
        y = random.uniform(-RANDOM_START_BOUND, RANDOM_START_BOUND)
        yaw = random.uniform(-math.pi, math.pi)
        dist_target = math.hypot(x, y)
        dist_c2 = math.hypot(x - PARKED_CAR2_POS[0], y - PARKED_CAR2_POS[1])
        dist_c3 = math.hypot(x - PARKED_CAR3_POS[0], y - PARKED_CAR3_POS[1])
        if (
            dist_target > RANDOM_START_MIN_DIST
            and dist_c2 > 0.9
            and dist_c3 > 0.9
        ):
            return [x, y], yaw


def apply_control(body_id, steer_joints, drive_joints, steer, speed):
    for j in steer_joints:
        p.setJointMotorControl2(
            body_id, j, p.POSITION_CONTROL, targetPosition=steer
        )
    for j in drive_joints:
        p.setJointMotorControl2(
            body_id, j, p.VELOCITY_CONTROL, targetVelocity=speed, force=100
        )


def has_collision(body_a, body_b):
    return len(p.getContactPoints(body_a, body_b)) > 0


# ---------------------------------------------------------------------------
# Rule-based scripted controller
# ---------------------------------------------------------------------------


def scripted_controller(pos, yaw, car2_xy, car3_xy, phase):
    """
    Stateful 3-phase controller.  Caller passes current phase (0/1/2) and
    receives updated (steer, speed, new_phase).

    Phase 0 - Navigate to APPROACH_WP (north of the spot)
    Phase 1 - Align heading to TARGET_YAW while at waypoint
    Phase 2 - Drive straight into the parking spot
    """
    dx = TARGET_POS[0] - pos[0]
    dy = TARGET_POS[1] - pos[1]
    dist = math.hypot(dx, dy)

    wp_dx = APPROACH_WP[0] - pos[0]
    wp_dy = APPROACH_WP[1] - pos[1]
    wp_dist = math.hypot(wp_dx, wp_dy)

    yaw_err_to_park = normalize_angle(TARGET_YAW - yaw)

    # One-way phase transitions
    if phase == 0 and wp_dist < 0.6:
        phase = 1
    if phase == 1 and abs(yaw_err_to_park) < 0.3:
        phase = 2

    if phase == 0:
        # Head to approach waypoint
        angle_to_wp = math.atan2(wp_dy, wp_dx)
        heading_err = normalize_angle(angle_to_wp - yaw)
        if abs(heading_err) <= math.pi / 2:
            direction, eff_err = 1.0, heading_err
        else:
            direction = -1.0
            eff_err = normalize_angle(heading_err - math.pi)
        steer = max(-0.4, min(0.4, eff_err * 1.4))
        speed = direction * min(7.0, max(1.5, wp_dist * 4.0))

    elif phase == 1:
        # Tight arc to align yaw to TARGET_YAW
        steer = max(-0.4, min(0.4, yaw_err_to_park * 3.0))
        speed = 1.5 if yaw_err_to_park > 0 else -1.5

    else:
        # Drive into the parking spot
        angle_to_tgt = math.atan2(dy, dx)
        heading_err = normalize_angle(angle_to_tgt - yaw)
        if abs(heading_err) <= math.pi / 2:
            direction, eff_err = 1.0, heading_err
        else:
            direction = -1.0
            eff_err = normalize_angle(heading_err - math.pi)
        steer = max(-0.4, min(0.4, eff_err * 1.8))
        speed = direction * min(5.0, max(0.5, dist * 5.0))

    # Obstacle avoidance (skip during yaw-alignment phase)
    if phase != 1:
        for obs_xy in [car2_xy, car3_xy]:
            odx = pos[0] - obs_xy[0]
            ody = pos[1] - obs_xy[1]
            odist = math.hypot(odx, ody)
            if odist < 1.0:
                repulsion_yaw = math.atan2(ody, odx)
                rep_error = normalize_angle(repulsion_yaw - yaw)
                weight = (1.0 - odist) * 0.5
                steer = max(-0.4, min(0.4, steer + rep_error * weight))

    return steer, speed, phase


# ---------------------------------------------------------------------------
# Observation builder
# ---------------------------------------------------------------------------


def build_obs(pos, yaw, vel, ang_vel_z, car2_xy, car3_xy):
    dx = TARGET_POS[0] - pos[0]
    dy = TARGET_POS[1] - pos[1]
    dist = math.hypot(dx, dy)
    angle_to_target = math.atan2(dy, dx)
    heading_error = normalize_angle(angle_to_target - yaw)
    yaw_error = normalize_angle(TARGET_YAW - yaw)

    return {
        "x": pos[0],
        "y": pos[1],
        "yaw": yaw,
        "vx": vel[0],
        "vy": vel[1],
        "omega": ang_vel_z,
        "dist_to_target": dist,
        "angle_to_target": angle_to_target,
        "heading_error": heading_error,
        "yaw_error": yaw_error,
        "obs2_dx": car2_xy[0] - pos[0],
        "obs2_dy": car2_xy[1] - pos[1],
        "obs3_dx": car3_xy[0] - pos[0],
        "obs3_dy": car3_xy[1] - pos[1],
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.2])
    car2 = p.loadURDF(
        "racecar/racecar.urdf", [PARKED_CAR2_POS[0], PARKED_CAR2_POS[1], 0.2]
    )
    car3 = p.loadURDF(
        "racecar/racecar.urdf", [PARKED_CAR3_POS[0], PARKED_CAR3_POS[1], 0.2]
    )

    steer_joints, drive_joints = find_joints(car)

    total_attempts = 0
    total_success = 0
    total_rows = 0

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        while (
            total_success < TARGET_SUCCESS_EPISODES
            and total_attempts < MAX_ATTEMPTS
        ):
            total_attempts += 1
            start_xy, start_yaw = random_start()
            reset_car(car, start_xy, start_yaw)

            episode_rows = []
            collided = False
            success = False
            phase = 0

            for step in range(MAX_STEPS):
                pos, yaw, vel, omega = get_state(car)
                c2_pos, _, _, _ = get_state(car2)
                c3_pos, _, _, _ = get_state(car3)
                car2_xy = [c2_pos[0], c2_pos[1]]
                car3_xy = [c3_pos[0], c3_pos[1]]

                obs = build_obs(pos, yaw, vel, omega, car2_xy, car3_xy)

                steer, speed, phase = scripted_controller(
                    pos, yaw, car2_xy, car3_xy, phase
                )

                # Add action noise for training diversity
                steer = max(
                    -0.4, min(0.4, steer + random.gauss(0, ACTION_NOISE_STEER))
                )
                speed = speed + random.gauss(0, ACTION_NOISE_SPEED)

                apply_control(car, steer_joints, drive_joints, steer, speed)

                for _ in range(PHYSICS_SUB_STEPS):
                    p.stepSimulation()

                # Check collision with parked cars
                if has_collision(car, car2) or has_collision(car, car3):
                    collided = True
                    break

                row = {
                    "episode": total_attempts,
                    "step": step,
                    **obs,
                    "action_steer": round(steer, 5),
                    "action_speed": round(speed, 5),
                    "success": 0,
                }  # filled in after episode
                episode_rows.append(row)

                # Re-read state AFTER physics step for success check
                pos_new, yaw_new, vel_new, _ = get_state(car)
                dist_new = math.hypot(
                    TARGET_POS[0] - pos_new[0], TARGET_POS[1] - pos_new[1]
                )
                yaw_err_new = abs(normalize_angle(TARGET_YAW - yaw_new))
                speed_mag = math.hypot(vel_new[0], vel_new[1])
                if (
                    dist_new < SUCCESS_DIST
                    and yaw_err_new < SUCCESS_YAW_ERR
                    and speed_mag < SUCCESS_SPEED
                ):
                    success = True
                    break

            if success:
                total_success += 1
                for row in episode_rows:
                    row["success"] = 1
                for row in episode_rows:
                    writer.writerow(row)
                total_rows += len(episode_rows)

            # Report progress
            if total_attempts % 100 == 0:
                rate = total_success / total_attempts * 100
                print(
                    f"  attempt {total_attempts:4d} | "
                    f"successes {total_success:4d}/{TARGET_SUCCESS_EPISODES} "
                    f"({rate:.0f}%) | rows {total_rows:,}"
                )

    p.disconnect()

    success_rate = total_success / max(total_attempts, 1) * 100
    print(
        f"\nFinished: {total_success} successful episodes "
        f"from {total_attempts} attempts ({success_rate:.0f}% success rate)"
    )
    print(f"Total rows written: {total_rows:,}")
    print(f"Data saved to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
