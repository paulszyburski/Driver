from smbus import SMBus

bus = SMBus(1)
MPU_ADDRESS = 0x68

# Wake the MPU-6050.
bus.write_byte_data(MPU_ADDRESS, 0x6B, 0)

# Gyroscope range: ±500 degrees per second.
bus.write_byte_data(MPU_ADDRESS, 0x1B, 0x08)

# Accelerometer range: ±2 g.
bus.write_byte_data(MPU_ADDRESS, 0x1C, 0x00)

GYRO_SCALE = 65.5       # LSB per degree/second at ±500°/s
ACCEL_SCALE = 16384.0   # LSB per g at ±2 g


def scan_with_ultrasonic(bot):
    distance = bot.requestUltrasonicSensor(1, 2, onDistance)
    return distance


def to_signed_16(high, low):
    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


def read_gyro():
    """Return gyroscope measurements in degrees per second."""
    data = bus.read_i2c_block_data(MPU_ADDRESS, 0x43, 6)

    gyro_x_raw = to_signed_16(data[0], data[1])
    gyro_y_raw = to_signed_16(data[2], data[3])
    gyro_z_raw = to_signed_16(data[4], data[5])

    gyro_x = gyro_x_raw / GYRO_SCALE
    gyro_y = gyro_y_raw / GYRO_SCALE
    gyro_z = gyro_z_raw / GYRO_SCALE

    return gyro_x, gyro_y, gyro_z


def read_accelerometer():
    """
    Return accelerometer measurements in g.

    When the module is stationary and lying flat, you should normally see:
        x ≈ 0 g
        y ≈ 0 g
        z ≈ 1 g

    The exact axis depends on the module's orientation.
    """
    data = bus.read_i2c_block_data(MPU_ADDRESS, 0x3B, 6)

    accel_x_raw = to_signed_16(data[0], data[1])
    accel_y_raw = to_signed_16(data[2], data[3])
    accel_z_raw = to_signed_16(data[4], data[5])

    accel_x = accel_x_raw / ACCEL_SCALE
    accel_y = accel_y_raw / ACCEL_SCALE
    accel_z = accel_z_raw / ACCEL_SCALE

    return accel_x, accel_y, accel_z


def normalize_angle(angle):
    """Convert an angle to the range -180 to 180."""
    return (angle + 180) % 360 - 180



    def drive_distance(self, distance):
        drive_time = distance / 14

        self.bot.doMove(100, 100)

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