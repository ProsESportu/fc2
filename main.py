import time

import ustruct
from machine import time_pulse_us, Pin, I2C, PWM
from micropython import const

from PID import PID
from interfaces import IDrives,IRCReceiver,IGyro
from utils import map_from_to

DEVICE_ADDRESS = const(0b1101011)


class RCReceiver(IRCReceiver):
    def __init__(self, pins: list[Pin | int], value_range: tuple[int, int] | None = None):
        if len(pins) < 1 or len(pins) > 6:
            raise Exception("wrong number of pins", pins)
        pins: list[Pin] = [Pin(i, Pin.IN) for i in pins if not isinstance(i, Pin)]
        self._pins = pins
        self._value_range = value_range

    def _get_raw(self) -> list[int]:
        return [time_pulse_us(i, 3_000_000) for i in self._pins]

    def get_mapped(self) -> list[float]:
        if self._value_range is not None:
            low, high = self._value_range
            return [map_from_to(i, low, high, -1, 1) for i in self._get_raw()]
        else:
            return self._get_raw()



class Gyro(IGyro):
    def __init__(self, slice_id: int, signal: Pin | int, clock: Pin | int, freq: int = 400_000):
        if not isinstance(signal, Pin):
            signal = Pin(signal)
        if not isinstance(clock, Pin):
            clock = Pin(clock)
        self._avg_x = 0
        self._avg_y = 0
        self._avg_z = 0
        self._i2c = I2C(slice_id, scl=clock, sda=signal, freq=freq)
        self._i2c.writeto_mem(DEVICE_ADDRESS, 0x20, 0b10001111.to_bytes())

    def _read(self):
        x_low = self._i2c.readfrom_mem(DEVICE_ADDRESS, 0x28, 2)
        # x_high = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x29, 1)[0]
        y_low = self._i2c.readfrom_mem(DEVICE_ADDRESS, 0x2a, 2)
        # y_high = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x2b, 1)[0]
        z_low = self._i2c.readfrom_mem(DEVICE_ADDRESS, 0x2c, 2)
        # z_high = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x2d, 1)[0]
        x: int = ustruct.unpack("<h", x_low)[0]
        y: int = ustruct.unpack("<h", y_low)[0]
        z: int = ustruct.unpack("<h", z_low)[0]

        # x = int.from_bytes(x_low, "little", signed=True)
        # y = int.from_bytes(y_low, "little", signed=True)
        # z = int.from_bytes(z_low, "little", signed=True)
        return x, y, z

    def calibrate(self, count=10_000):
        x_sum = 0
        y_sum = 0
        z_sum = 0
        for i in range(count):
            x, y, z = self._read()
            x_sum += x
            y_sum += y
            z_sum += z
        self._avg_x = x_sum // count
        self._avg_y = y_sum // count
        self._avg_z = z_sum // count

    def _read_calibrated(self) -> tuple[int, int, int]:
        x, y, z = self._read()
        return x - self._avg_x, y - self._avg_y, z - self._avg_z

    def read_calibrated_mapped(self) -> tuple[float, float, float]:
        x, y, z = self._read_calibrated()
        return (
            map_from_to(x, -32768, 32767, -1, 1),
            map_from_to(y, -32768, 32767, -1, 1),
            map_from_to(z, -32768, 32767, -1, 1),
        )


class Drives(IDrives):
    def __init__(self, front_left: Pin | int, front_right: Pin | int, rear_right: Pin | int, rear_left: Pin | int,
                 freq: int = 50):
        self._front_left = PWM(front_left, freq=freq)
        self._front_right = PWM(front_right, freq=freq)
        self._rear_right = PWM(rear_right, freq=freq)
        self._rear_left = PWM(rear_left, freq=freq)

    def set(self, yaw: float, pitch: float, roll: float, throttle: float):
        front_left_power = throttle + pitch + roll - yaw
        front_right_power = throttle + pitch - roll + yaw
        rear_right_power = throttle - pitch - roll - yaw
        rear_left_power = throttle - pitch + roll + yaw
        self._front_left.duty_u16(int(front_left_power * 65535))
        self._front_right.duty_u16(int(front_right_power * 65535))
        self._rear_right.duty_u16(int(rear_right_power * 65535))
        self._rear_left.duty_u16(int(rear_left_power * 65535))


current_yaw: float = 0
current_pitch: float = 0
current_roll: float = 0
gyro = Gyro(1, 0, 1)
gyro.calibrate()
CYCLE_FREQ = const(400)
CYCLE_TIME = const(1 / CYCLE_FREQ)
drives = Drives(0, 1, 2, 3)
pid_yaw = PID(1, 0, 0, CYCLE_TIME, 1)
pid_pitch = PID(1, 0, 0, CYCLE_TIME, 1)
pid_roll = PID(1, 0, 0, CYCLE_TIME, 1)
receiver = RCReceiver([4, 5, 6, 7], (1_000, 2_000))
while True:
    loop_start = time.ticks_us()
    yaw_change, pitch_change, roll_change = gyro.read_calibrated_mapped()
    current_yaw += yaw_change
    current_pitch += pitch_change
    current_roll += roll_change
    [desired_yaw, desired_pitch, desired_roll, desired_throttle] = receiver.get_mapped()
    yaw_power = pid_yaw.update(desired_yaw, current_yaw)
    pitch_power = pid_pitch.update(desired_pitch, current_pitch)
    roll_power = pid_roll.update(desired_roll, current_roll)
    drives.set(yaw_power, pitch_power, roll_power, desired_throttle)
    loop_end = time.ticks_us()
    loop_duration = time.ticks_diff(loop_end, loop_start)
    if loop_duration < CYCLE_TIME * 1_000_000:
        time.sleep_us(int(CYCLE_TIME * 1_000_000 - loop_duration))
