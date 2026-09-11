import math
from time import perf_counter, sleep

import cv2
import numpy as np

from .camera import Camera
from .utils import normalize_angle, read_gyro, read_accelerometer
from .MBotlib.mBot import mBot


class BotController:
    def __init__(self, bot):
        self.bot = bot

        self.curr_angle = 0.0
        self.position = (0.0, 0.0)

        print("Keep the robot still while calibrating the gyroscope...")
        self.gyro_z_bias = self.calibrate_gyro_z()
        

    def calibrate_gyro_z(self, samples=300):
        total = 0.0

        for _ in range(samples):
            _, _, gyro_z = read_gyro()
            total += gyro_z
            sleep(0.003)

        return total / samples


    def rotate(self, degrees, acceptable_error=1):
        if abs(degrees) <= acceptable_error:
            return

        direction = 1 if degrees > 0 else -1
        target_angle = abs(degrees)
        measured_angle = 0.0

        current_speed = None
        previous_time = perf_counter()

        try:
            while measured_angle < target_angle - acceptable_error:
                remaining = target_angle - measured_angle

                if remaining > 30:
                    speed = 150
                else:
                    speed = 120

                if speed != current_speed:
                    self.bot.doMove(
                        direction * speed,
                        -direction * speed
                    )
                    current_speed = speed

                _, _, gyro_z = read_gyro()

                current_time = perf_counter()
                dt = current_time - previous_time
                previous_time = current_time

                corrected_gyro_z = gyro_z - self.gyro_z_bias

                measured_angle += abs(corrected_gyro_z) * dt

                sleep(0.001)

        finally:
            self.bot.doMove(0, 0)

        self.curr_angle = normalize_angle(
            self.curr_angle + direction * measured_angle
        )

    def drive_distance(self, distance):
        speed = 100
        drive_time = distance / (14 * speed / 100)

        self.bot.doMove(speed, speed)

        try:
            sleep(drive_time)
        finally:
            self.bot.doMove(0, 0)

        angle_radians = math.radians(self.curr_angle)

        delta_x = math.sin(angle_radians) * distance
        delta_y = math.cos(angle_radians) * distance

        current_x, current_y = self.position

        self.position = (
            current_x + delta_x,
            current_y + delta_y
        )

    def move_to(self, x, y):
        current_x, current_y = self.position

        dx = x - current_x
        dy = y - current_y

        distance = math.hypot(dx, dy)

        if distance <= 0.1:
            return

        target_angle = math.degrees(
            math.atan2(dx, dy)
        )

        turn_angle = normalize_angle(
            target_angle - self.curr_angle
        )

        print(
            f"Moving {distance:.2f} cm at "
            f"{target_angle:.2f}°"
        )

        self.rotate(turn_angle)
        sleep(0.5)
        self.drive_distance(distance)

        self.position = (x, y)

if __name__ == "__main__":
    bot = mBot()
    bot.startWithSerial("/dev/ttyUSB0")

    sleep(2)

    controller = BotController(bot)

    controller.drive_distance(10)