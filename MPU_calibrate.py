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
            print(self.read_mpu()[3:])

    def read_mpu(self):
        accel_x, accel_y, accel_z = self.mpu.get_accel_data().values()
        rot_x, rot_y, rot_z = self.mpu.get_gyro_data().values()
        return (accel_x, accel_y, accel_z,
                rot_x - self.MPU_X_ROTATION_OFFSET,
                rot_y - self.MPU_Y_ROTATION_OFFSET,
                rot_z - self.MPU_Z_ROTATION_OFFSET)


if __name__ == "__main__":
    drone = Drone()
    drone.calibrate()
