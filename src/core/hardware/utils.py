from smbus import SMBus

bus = SMBus(1)
MPU_ADDRESS = 0x68

bus.write_byte_data(MPU_ADDRESS, 0x6B, 0)
bus.write_byte_data(MPU_ADDRESS, 0x1B, 0x08)
bus.write_byte_data(MPU_ADDRESS, 0x1C, 0x00)

GYRO_SCALE = 65.5
ACCEL_SCALE = 16384.0


def scan_with_ultrasonic(bot):
    return bot.requestUltrasonicSensor(1, 2)


def to_signed_16(high, low):
    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


def read_gyro():
    data = bus.read_i2c_block_data(MPU_ADDRESS, 0x43, 6)

    gyro_x_raw = to_signed_16(data[0], data[1])
    gyro_y_raw = to_signed_16(data[2], data[3])
    gyro_z_raw = to_signed_16(data[4], data[5])

    gyro_x = gyro_x_raw / GYRO_SCALE
    gyro_y = gyro_y_raw / GYRO_SCALE
    gyro_z = gyro_z_raw / GYRO_SCALE

    return gyro_x, gyro_y, gyro_z


def read_accelerometer():
    data = bus.read_i2c_block_data(MPU_ADDRESS, 0x3B, 6)

    accel_x_raw = to_signed_16(data[0], data[1])
    accel_y_raw = to_signed_16(data[2], data[3])
    accel_z_raw = to_signed_16(data[4], data[5])

    accel_x = accel_x_raw / ACCEL_SCALE
    accel_y = accel_y_raw / ACCEL_SCALE
    accel_z = accel_z_raw / ACCEL_SCALE

    return accel_x, accel_y, accel_z


def normalize_angle(angle):
    return (angle + 180) % 360 - 180
