import smbus
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
