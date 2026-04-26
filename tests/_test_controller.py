import src.generate_data as gd, pybullet as p, pybullet_data, math
from src.utils import find_joints

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.2])
car2 = p.loadURDF("racecar/racecar.urdf", [0.5, 0, 0.2])
car3 = p.loadURDF("racecar/racecar.urdf", [-0.5, 0, 0.2])
steer_j, drive_j = find_joints(car)

successes = 0
for ep in range(30):
    xy, yaw = gd.random_start()
    gd.reset_car(car, xy, yaw)
    final_d, final_ye = 99.0, 99.0
    phase = 0
    for step in range(gd.MAX_STEPS):
        pos, yaw_c, vel, omega = gd.get_state(car)
        c2, _, _, _ = gd.get_state(car2)
        c3, _, _, _ = gd.get_state(car3)
        steer, speed, phase = gd.scripted_controller(
            pos, yaw_c, [c2[0], c2[1]], [c3[0], c3[1]], phase
        )
        gd.apply_control(car, steer_j, drive_j, steer, speed)
        for _ in range(gd.PHYSICS_SUB_STEPS):
            p.stepSimulation()
        pos2, yaw2, vel2, _ = gd.get_state(car)
        d = math.hypot(gd.TARGET_POS[0] - pos2[0], gd.TARGET_POS[1] - pos2[1])
        ye = abs(gd.normalize_angle(gd.TARGET_YAW - yaw2))
        sm = math.hypot(vel2[0], vel2[1])
        final_d, final_ye = d, ye
        if (
            d < gd.SUCCESS_DIST
            and ye < gd.SUCCESS_YAW_ERR
            and sm < gd.SUCCESS_SPEED
        ):
            print(
                f"Ep {ep:2d}: SUCCESS step={step:4d}  dist={d:.3f}  ye={ye:.3f}"
            )
            successes += 1
            break
    else:
        print(
            f"Ep {ep:2d}: TIMEOUT           dist={final_d:.3f}  ye={final_ye:.3f}"
        )

p.disconnect()
print(f"\n{successes}/30 succeeded")
