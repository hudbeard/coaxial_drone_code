import smbus
import mpu6050


class Drone(object):
    def __init__(self):
        self.MPU_X_ROTATION_OFFSET = 0
        self.MPU_Y_ROTATION_OFFSET = 0
        self.MPU_Z_ROTATION_OFFSET = 0

        self.mpu = mpu6050.mpu6050(0x68)

    def calibrate(self):
        while True:
            print(self.mpu.get_gyro_data().values())


if __name__ == "__main__":
    drone = Drone()
    drone.calibrate()
