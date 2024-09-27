import random
import sys

from PID import PID
from interfaces import IDrives, IRCReceiver, IGyro
import matplotlib.pyplot as plt
import matplotlib.widgets as wgt
import time


class Drives(IDrives):
    def __init__(self, fig):
        ((front_left, front_right), (back_left, back_right)) = fig.subplots(2, 2)
        self.front_left = front_left
        self.front_right = front_right
        self.back_left = back_left
        self.back_right = back_right
        self.front_left_his = []
        self.front_right_his = []
        self.back_left_his = []
        self.back_right_his = []
        self.counter = 0
        # plt.ion()
        # plt.show()

    def set(self, yaw, pitch, roll, throttle):
        front_left_power = throttle + pitch + roll - yaw
        front_right_power = throttle + pitch - roll + yaw
        rear_right_power = throttle - pitch - roll - yaw
        rear_left_power = throttle - pitch + roll + yaw
        # self.front_left.set_linestyle('dashed')
        self.front_left_his.append(front_left_power)
        self.front_right_his.append(front_right_power)
        self.back_left_his.append(rear_left_power)
        self.back_right_his.append(rear_right_power)
        self.front_left.plot(range(0, len(self.front_left_his)), self.front_left_his, linestyle='-', color='red',
                             linewidth=2)

        self.front_right.plot(range(0, len(self.front_left_his)), self.front_right_his, linestyle='-', color='red',
                              linewidth=2)
        self.back_left.plot(range(0, len(self.front_left_his)), self.back_left_his, linestyle='-', color='red',
                            linewidth=2)
        self.back_right.plot(range(0, len(self.front_left_his)), self.back_right_his, linestyle='-', color='red',
                             linewidth=2)
        self.counter += 1


class RCReceiver(IRCReceiver):
    def __init__(self, fig):
        yaw_ax, pitch_ax, roll_ax, throttle_ax = fig.subplots(4)
        yaw = wgt.Slider(
            ax=yaw_ax,
            label='Yaw',
            valmin=-1,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )
        pitch = wgt.Slider(
            ax=pitch_ax,
            label='Pitch',
            valmin=-1,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )
        roll = wgt.Slider(
            ax=roll_ax,
            label='Roll',
            valmin=-1,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )
        throttle = wgt.Slider(
            ax=throttle_ax,
            label='Throttle',
            valmin=0,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.throttle = throttle

    def get_mapped(self):
        return [self.yaw.val, self.pitch.val, self.roll.val, self.throttle.val]


class Gyro(IGyro):
    def __init__(self, fig):
        ax, ay, az = fig.subplots(3)
        self.x = wgt.Slider(
            ax=ax,
            label='X',
            valmin=-1,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )
        self.y = wgt.Slider(
            ax=ay,
            label='Y',
            valmin=-1,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )
        self.z = wgt.Slider(
            ax=az,
            label='Z',
            valmin=-1,
            valmax=1,
            valinit=0,
            orientation='horizontal',
        )

    def read_calibrated_mapped(self):
        return self.x.val, self.y.val, self.z.val


with plt.ion():
    fig = plt.figure(figsize=(15, 5))
    (rc_fig, gyro_fig, drives_fig) = fig.subfigures(1, 3, width_ratios=[0.5, 0.5, 1])
    drives = Drives(drives_fig)
    rcreceiver = RCReceiver(rc_fig)
    gyro = Gyro(gyro_fig)
    CYCLE_FREQ = 400
    CYCLE_TIME = 1 / CYCLE_FREQ
    pid_yaw = PID(1, 0, 0, CYCLE_TIME, 1)
    pid_pitch = PID(1, 0, 0, CYCLE_TIME, 1)
    pid_roll = PID(1, 0, 0, CYCLE_TIME, 1)

    while True:
        start = time.perf_counter()
        desired_yaw, desired_pitch, desired_roll, desired_throttle = rcreceiver.get_mapped()
        yaw_change, pitch_change, roll_change = gyro.read_calibrated_mapped()
        target_yaw = pid_yaw.update(desired_yaw, yaw_change)
        target_pitch = pid_pitch.update(desired_pitch, pitch_change)
        target_roll = pid_roll.update(desired_roll, roll_change)
        drives.set(target_yaw, target_pitch, target_roll, desired_throttle)
        fig.canvas.draw()
        fig.canvas.flush_events()
        el = time.perf_counter() - start
        rem = CYCLE_TIME - el
        print(rem)
        time.sleep(rem if rem > 0 else 0)
