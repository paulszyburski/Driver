import math
import time
from smbus import SMBus


MPU_ADDRESS = 0x68
I2C_BUS = 1

PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

ACCEL_XOUT_H = 0x3B

ACCEL_SCALE = 16384.0  # ±2 g
GYRO_SCALE = 131.0  # ±250 degrees/second

# Higher values trust the gyro more.
# Lower values correct tilt more strongly using the accelerometer.
FILTER_ALPHA = 0.98


def signed_16_bit(high: int, low: int) -> int:
    value = (high << 8) | low

    if value >= 0x8000:
        value -= 65536

    return value


class MPU6050:
    def __init__(self, bus_number: int = I2C_BUS, address: int = MPU_ADDRESS):
        self.bus = SMBus(bus_number)
        self.address = address

        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0

        self.initialize()

    def initialize(self) -> None:
        # Wake the MPU-6050.
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)

        # Digital low-pass filter.
        self.bus.write_byte_data(self.address, CONFIG, 0x03)

        # Gyroscope range: ±250 degrees/second.
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)

        # Accelerometer range: ±2 g.
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)

        time.sleep(1.0)

        # Discard initial unstable readings.
        for _ in range(100):
            self.read_raw()
            time.sleep(0.005)

    def read_raw(self) -> tuple[float, float, float, float, float, float]:
        # Read accelerometer, temperature and gyro registers together.
        data = self.bus.read_i2c_block_data(
            self.address,
            ACCEL_XOUT_H,
            14,
        )

        accel_x_raw = signed_16_bit(data[0], data[1])
        accel_y_raw = signed_16_bit(data[2], data[3])
        accel_z_raw = signed_16_bit(data[4], data[5])

        gyro_x_raw = signed_16_bit(data[8], data[9])
        gyro_y_raw = signed_16_bit(data[10], data[11])
        gyro_z_raw = signed_16_bit(data[12], data[13])

        accel_x = accel_x_raw / ACCEL_SCALE
        accel_y = accel_y_raw / ACCEL_SCALE
        accel_z = accel_z_raw / ACCEL_SCALE

        gyro_x = gyro_x_raw / GYRO_SCALE
        gyro_y = gyro_y_raw / GYRO_SCALE
        gyro_z = gyro_z_raw / GYRO_SCALE

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def calibrate_gyro(self, samples: int = 1000) -> None:
        print("Keep the sensor completely still while calibrating...")

        total_x = 0.0
        total_y = 0.0
        total_z = 0.0

        for sample_number in range(samples):
            _, _, _, gyro_x, gyro_y, gyro_z = self.read_raw()

            total_x += gyro_x
            total_y += gyro_y
            total_z += gyro_z

            if sample_number % 100 == 0:
                progress = sample_number / samples * 100
                print(f"\rCalibration: {progress:5.1f}%", end="", flush=True)

            time.sleep(0.003)

        self.gyro_offset_x = total_x / samples
        self.gyro_offset_y = total_y / samples
        self.gyro_offset_z = total_z / samples

        print("\rCalibration: 100.0%")
        print(
            "Gyro offsets: "
            f"X={self.gyro_offset_x:.3f}, "
            f"Y={self.gyro_offset_y:.3f}, "
            f"Z={self.gyro_offset_z:.3f} °/s"
        )

    def read(self) -> tuple[float, float, float, float, float, float]:
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_raw()

        gyro_x -= self.gyro_offset_x
        gyro_y -= self.gyro_offset_y
        gyro_z -= self.gyro_offset_z

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def close(self) -> None:
        self.bus.close()


def normalize_angle(angle: float) -> float:
    """Keep an angle between -180 and +180 degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def accelerometer_angles(
    accel_x: float,
    accel_y: float,
    accel_z: float,
) -> tuple[float, float]:
    roll = math.degrees(math.atan2(accel_y, accel_z))

    pitch = math.degrees(
        math.atan2(
            -accel_x,
            math.sqrt(accel_y**2 + accel_z**2),
        )
    )

    return roll, pitch


def main() -> None:
    sensor = MPU6050()

    try:
        sensor.calibrate_gyro()

        accel_x, accel_y, accel_z, _, _, _ = sensor.read()

        # Roll and pitch can be initialized using gravity.
        roll, pitch = accelerometer_angles(accel_x, accel_y, accel_z)

        # The MPU-6050 cannot determine absolute compass heading.
        # Therefore yaw begins at zero.
        yaw = 0.0

        previous_time = time.monotonic()

        print("\nOrientation tracking started.")
        print("Yaw is relative to the program's starting position.")
        print("Press Ctrl+C to stop.\n")

        while True:
            current_time = time.monotonic()
            delta_time = current_time - previous_time
            previous_time = current_time

            # Prevent one unusually long pause from causing a huge angle jump.
            if delta_time <= 0 or delta_time > 0.2:
                time.sleep(0.005)
                continue

            (
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
            ) = sensor.read()

            accel_roll, accel_pitch = accelerometer_angles(
                accel_x,
                accel_y,
                accel_z,
            )

            # Complementary filter:
            # gyro provides fast movement;
            # accelerometer corrects long-term roll/pitch drift.
            roll = (
                FILTER_ALPHA * (roll + gyro_x * delta_time)
                + (1.0 - FILTER_ALPHA) * accel_roll
            )

            pitch = (
                FILTER_ALPHA * (pitch + gyro_y * delta_time)
                + (1.0 - FILTER_ALPHA) * accel_pitch
            )

            # Yaw only uses the gyro, so it will gradually drift.
            yaw += gyro_z * delta_time

            roll = normalize_angle(roll)
            pitch = normalize_angle(pitch)
            yaw = normalize_angle(yaw)

            print(
                f"\rRoll: {roll:8.2f}°  Pitch: {pitch:8.2f}°  Yaw: {yaw:8.2f}°  ",
                end="",
                flush=True,
            )

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nStopped.")

    except OSError as error:
        print(f"\nI²C error: {error}")

    finally:
        sensor.close()


if __name__ == "__main__":
    main()
