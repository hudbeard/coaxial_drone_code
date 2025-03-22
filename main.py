import smbus
import os
import time
import math
from pathlib import Path
# import gps
# import demo_opts
from luma.core.render import canvas
from PIL import ImageFont
import mpu6050
import serial
import struct

os.system("sudo pigpiod")
time.sleep(1)
import pigpio


class Drone(object):
    def __init__(self):
        # Init ECS's
        self.THRUST_ESC_PIN = 26
        self.DIRECTION_ESC_PIN = 16
        self.pi = pigpio.pi()
        self.ESC_MAX_VALUE = 2000
        self.ESC_MIN_VALUE = 1000

        self.MPU_X_ROTATION_OFFSET = 0.5
        self.MPU_Y_ROTATION_OFFSET = 6.3
        self.MPU_Z_ROTATION_OFFSET = 1.5
        self.ROTATION_SCALE_FACTOR = 1000

        self.configure_escs()

        # Init Magnetic Encoder
        self.ENCODER_ADDRESS = 0x36
        self.bus = smbus.SMBus(1)

        # Init GPS
        # self.gpsd = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

        # Init display
        # self.display = demo_opts.get_device()

        # Init gyro
        self.mpu = mpu6050.mpu6050(0x68)

        # Init Receiver
        self.ibus = IBus("/dev/serial0")

        self.thrust_motor_power = 1500
        self.direction_motor_power = 1500

    def configure_escs(self):
        self.pi.set_servo_pulsewidth(self.THRUST_ESC_PIN, 0)
        self.pi.set_servo_pulsewidth(self.DIRECTION_ESC_PIN, 0)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(self.THRUST_ESC_PIN, self.ESC_MAX_VALUE)
        self.pi.set_servo_pulsewidth(self.DIRECTION_ESC_PIN, self.ESC_MAX_VALUE)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(self.THRUST_ESC_PIN, self.ESC_MIN_VALUE)
        self.pi.set_servo_pulsewidth(self.DIRECTION_ESC_PIN, self.ESC_MIN_VALUE)

    def get_lat_long(self):
        report = {'class': None}
        while report['class'] != 'TPV':
            try:
                report = next(self.gpsd)
            except KeyError:
                pass

        return report.get('lat', None), report.get('lon', None), report.get('time', None)

    def read_motor_angle(self):
        read_bytes = self.bus.read_i2c_block_data(self.ENCODER_ADDRESS, 0x0C, 2)
        return (((read_bytes[0] << 8) | read_bytes[1]) / 4096) * 360

    def read_mpu(self):
        accel_x, accel_z, accel_y = self.mpu.get_accel_data().values()
        rot_x, rot_z, rot_y = self.mpu.get_gyro_data().values()
        return (accel_x, accel_y, accel_z,
                round(rot_x - self.MPU_X_ROTATION_OFFSET, ndigits=2),
                round(rot_y - self.MPU_Y_ROTATION_OFFSET, ndigits=2),
                round(rot_z - self.MPU_Z_ROTATION_OFFSET, ndigits=2))

    def display_stats(self):
        font_path = str(Path(__file__).resolve().parent.joinpath('fonts', 'SegoeIcons.ttf'))
        font2_path = str(Path(__file__).resolve().parent.joinpath('fonts', 'C&C Red Alert [INET].ttf'))
        font = ImageFont.truetype(font_path, 16)
        font2 = ImageFont.truetype(font2_path, 12)
        font3 = ImageFont.truetype(font2_path, 8)
        lat, long, gps_time = self.get_lat_long()

        with canvas(self.display) as draw:
            draw.text((0, 0), "             ", font=font, fill="white")
            draw.text((20, 8), "12.5v", font=font3, fill="white")
            draw.text((0, 16), f"Lat    : {lat}", font=font2, fill="white")
            draw.text((0, 32), f"Long  : {long}", font=font2, fill="white")
            draw.text((0, 48), f"Time    : {gps_time}", font=font2, fill="white")

    def send_to_motors(self):
        self.pi.set_servo_pulsewidth(self.THRUST_ESC_PIN, self.thrust_motor_power)
        self.pi.set_servo_pulsewidth(self.DIRECTION_ESC_PIN, self.direction_motor_power)

    def get_controls(self):
        """
        Takes controller flight values and normalize them.
        :return: normalized values
        """
        _, _, roll, pitch, thrust, yaw, left_knob, right_knob, *_ = self.ibus.read()
        normalized_roll = (roll - 1500) / 500
        normalized_pitch = (pitch - 1500) / 500
        normalized_yaw = (yaw - 1500) / 500
        normalized_thrust = (thrust - 1000) / 1000
        return normalized_roll, normalized_pitch, normalized_yaw, normalized_thrust

    def stabilize(self, roll_rate, pitch_rate, yaw_rate):
        rotation_rate = roll_rate ** 2 + pitch_rate ** 2 / self.ROTATION_SCALE_FACTOR
        rotation_rate = 1000 if rotation_rate > 1000 else rotation_rate
        rotation_direction = math.degrees(math.atan(roll_rate / pitch_rate))
        direction_power = (math.sin(math.radians(self.read_motor_angle() - rotation_direction)) + 1) * (rotation_rate / 2)

    def convert_to_esc(self, val: float, _type="0:1"):
        if _type == "-1:1":
            return (self.ESC_MAX_VALUE - self.ESC_MIN_VALUE) * val
        else:
            return (self.ESC_MAX_VALUE - self.ESC_MIN_VALUE) * val + self.ESC_MIN_VALUE

    def run(self):
        try:
            self.configure_escs()
            while True:
                roll, pitch, yaw, thrust = self.get_controls()
                self.thrust_motor_power = self.convert_to_esc(thrust) + self.convert_to_esc(roll, _type="-1:1") * 0.5
                self.direction_motor_power = self.convert_to_esc(thrust) - self.convert_to_esc(roll, _type="-1:1") * 0.5
                self.send_to_motors()
        except KeyboardInterrupt:
            self.thrust_motor_power = 0
            self.direction_motor_power = 0
            self.send_to_motors()
            self.pi.stop()


class IBus:
    IBUS_START = b'\x20'
    IBUS_START_BYTES = [0x20, 0x40]
    IBUS_FORMAT = '<BBHHHHHHHHHHHHHHh'
    IBUS_FORMAT_CALC_CHECKSUM = '<BBHHHHHHHHHHHHHH'

    def __init__(self, serial_port: str, baudrate: int = 115200) -> None:
        self.port = serial_port
        self.baudrate = baudrate
        self.serial = serial.Serial(self.port, self.baudrate)

    def read(self) -> tuple:
        data = self.serial.read(32)

        while not self.validate(data):
            data = self.serial.read(1)

            while data != self.IBUS_START:
                data = self.serial.read(1)

            data += self.serial.read(31)

        if self.validate(data):
            return self.unpack(data)
        else:
            return ("err",)

    def unpack(self, data: tuple) -> tuple:
        if len(data) != 32:
            raise ValueError('Data length must be 32')

        return struct.unpack(self.IBUS_FORMAT, data)

    def validate(self, data: tuple) -> bool:
        data = self.unpack(data)

        return data[0] == 32 and data[1] == 64 and data[-1] == self.calc_checksum(data[:-1])

    def calc_checksum(self, data: list) -> int:
        return ((sum(bytearray(struct.pack(self.IBUS_FORMAT_CALC_CHECKSUM, *data)))) * -1) - 1


if __name__ == "__main__":
    drone = Drone()
    drone.run()
