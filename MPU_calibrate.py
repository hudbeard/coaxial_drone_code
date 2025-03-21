"""import smbus
from MPU6050 import MPU6050


class Drone(object):
    def __init__(self):
        self.MPU_X_ROTATION_OFFSET = 0
        self.MPU_Y_ROTATION_OFFSET = 0
        self.MPU_Z_ROTATION_OFFSET = 0

        self.mpu = MPU6050.mpu6050(0x68)
        self.mpu.reset()
        self.mpu.power_manage()
        self.mpu.gyro_config()
        self.mpu.accel_config()

    def calibrate(self):
        while True:
            print(self.mpu.read_gyroscope())


if __name__ == "__main__":
    drone = Drone()
    drone.calibrate()
"""

import smbus
import time

MPU6050_ADDR = 0x68  # Default I2C address

bus = smbus.SMBus(1)  # Use 1 for Raspberry Pi


def read_word(adr):
    high = bus.read_byte_data(MPU6050_ADDR, adr)
    low = bus.read_byte_data(MPU6050_ADDR, adr + 1)
    value = (high << 8) + low
    return value if value < 32768 else value - 65536  # Convert to signed


def read_sensor_data():
    ax = read_word(0x3B)
    ay = read_word(0x3D)
    az = read_word(0x3F)
    temp = read_word(0x41)
    gx = read_word(0x43)
    gy = read_word(0x45)
    gz = read_word(0x47)

    print(f"Ax: {ax}, Ay: {ay}, Az: {az}")
    print(f"Temp: {temp / 340.00 + 36.53:.2f}°C")
    print(f"Gx: {gx}, Gy: {gy}, Gz: {gz}")


# Wake up the sensor (it starts in sleep mode)
bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)

while True:
    read_sensor_data()
    time.sleep(1)
