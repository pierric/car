import time
import os
import logging
from collections import defaultdict
import math

import coloredlogs
import pygame

from PCA9685 import PCA9685


def compute_wheel_pwm(axis_x, axis_y):
    EPSILON = 1e-5
    logging.debug(f"{axis_x}, {axis_y}")

    if abs(axis_x) < 1e-1 and abs(axis_y) < 1e-1:
        return (0, 0)

    # y axis is slightly biased to 3e-5
    axis_y += 1e-4

    # theta is the angle between y and x direction.
    # goal to get the speed
    # when theta == 0:  the faster = 1, the slower = 1
    # when theta == 45: the faster ~ 0.9, the slower ~ 0
    # when theta == 90: the faster ~ 0.8, the slower ~ -0.5
    cos_theta = (abs(axis_y) + EPSILON) / (math.sqrt(axis_y * axis_y + axis_x * axis_x) + EPSILON)
    mag1 = 0.2 * cos_theta + 0.8
    mag2 = 1.5 * cos_theta - 0.5

    if axis_x < 0 and axis_y >= 0:
        mag_l = -mag2
        mag_r = -mag1
    elif axis_x >= 0 and axis_y >= 0:
        mag_l = -mag1
        mag_r = -mag2
    elif axis_x < 0 and axis_y < 0:
        mag_l = mag2
        mag_r = mag1
    elif axis_x >= 0 and axis_y < 0:
        mag_l = mag1
        mag_r = mag2

    logging.debug(f"mag_l:{mag_l}, mag_r:{mag_r}")
    return (mag_l, mag_r)


def compute_cam_pos(axis_x, axis_y):
    #logging.debug(f"{axis_x}, {axis_y}")
    return (axis_x, -axis_y)


class PWMController:
    MAX_ANGLE = 60
    EMA_ALPHA = 0.5

    def __init__(self):
        self.driver = PCA9685(1, 0x40)
        self.driver.setPWMFreq(50)
        self.ema = defaultdict(int)

    def update_wheel(self, pins, speed):
        (spin, fpin, bpin) = pins

        if abs(speed) < 1e-1:
            speed = 0

        if speed > 0:
            fval, bval = 1, 0
        elif speed < 0:
            fval, bval = 0, 1
        else:
            fval, bval = 0, 0

        sval = int(abs(speed) * 99)

        self.driver.setLevel(fpin, fval)
        self.driver.setLevel(bpin, bval)
        self.driver.setDutycycle(spin, sval)

    def update_servo(self, pin, angle):
        assert angle >= -1 and angle <= 1

        # angle < 5 degree
        if abs(angle) < 0.05:
            angle = 0

        # ema for sharp rotation
        if abs(self.ema[pin] - angle) > 0.1:
            angle = self.ema[pin] * self.EMA_ALPHA + angle * (1 - self.EMA_ALPHA)

        self.ema[pin] = angle

        rot_angle = (angle * 0.5 * self.MAX_ANGLE / 90 + 0.5) * 180
        self.driver.setAngle(pin, rot_angle)


def main(logging_level, interval):
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    root_logger = logging.getLogger()
    coloredlogs.install(
        level=logging_level,
        fmt="[%(asctime)s][%(funcName)s][%(levelname)s] - %(message)s",
        level_styles={
            "critical": {"bold": True, "color": "red"},
            "debug": {"color": "green"},
            "error": {"color": "red"},
            "info": {},
            "notice": {"color": "magenta"},
            "spam": {"color": "green", "faint": True},
            "success": {"bold": True, "color": "green"},
            "verbose": {"color": "blue"},
            "warning": {"color": "red"},
        },
        reconfigure=True,
    )

    pwm_ctrl = PWMController()

    pygame.init()
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    except:
        raise RuntimeError("Controller Not Intialised")

    while True:
        pygame.event.get()
        lsb_x = joystick.get_axis(0)
        lsb_y = joystick.get_axis(1)
        rsb_x = joystick.get_axis(3)
        rsb_y = joystick.get_axis(4)
        btn_a = joystick.get_button(0)

        speed_l, speed_r = compute_wheel_pwm(lsb_x, lsb_y)

        pwm_ctrl.update_wheel((0, 2, 1), speed_l)
        pwm_ctrl.update_wheel((4, 6, 5), speed_r)

        (ang_left_right, ang_up_down) = compute_cam_pos(rsb_x, rsb_y)
        pwm_ctrl.update_servo(15, ang_left_right)
        pwm_ctrl.update_servo(14, ang_up_down)

        time.sleep(interval)


if __name__ == "__main__":
    main(logging.INFO, 0.2)
