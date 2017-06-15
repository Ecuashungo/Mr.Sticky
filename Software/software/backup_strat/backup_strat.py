#!/usr/bin/env python
# license removed for brevity

import numpy as np
import math
import sys
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
import parameters as param
import rospy

# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes
NOSPEED = 0
NOT_VALID = -1


def get_conveyor_speed(conv_direction):
    lw_speed = NOSPEED
    rw_speed = NOSPEED
    cv_speed = np.sign(conv_direction) * param.get_conv_speed_control()
    speeds = [lw_speed, rw_speed, cv_speed]

    return limit_full_throttle(speeds)


def limit_full_throttle(speeds):
    upper_lim = param.get_full_throttle_control()
    lower_lim = - param.get_full_throttle_control()

    # for security reason, hard code arduino limits here
    if upper_lim > 255:
        upper_lim = 255
    if lower_lim < - 255:
        lower_lim = - 255

    for i in range(len(speeds)):
        if speeds[i] > upper_lim:
            speeds[i] = upper_lim
        elif speeds[i] < lower_lim:
            speeds[i] = lower_lim
    return speeds



class Control:
    def __init__(self):
        self.desired_conv_direction = NOSPEED

        self.game_start = None

        self.old_speed_x = NOSPEED
        self.old_speed_y = NOSPEED
        self.old_speed_theta = NOSPEED

        self.bottle_found = False
        self.x_angle_to_bottle = None
        return

    def cb_bdetector(self, data):
        num_bottles = data.num_bottles
        self.bottle_found = num_bottles != 0
        if (num_bottles):
            self.x_angle_to_bottle = data.final_x_angle
        return

    def cb_ir_sensors(self, data):

        return

    def calculate_speeds_to_bottle(self):
        # 0.79 = 45° / 180° * PI
        ANGLE_MAX = 0.79
        BASE_SPEED = 100
        speed_increment = math.fabs(ANGLE_MAX - self.x_angle_to_bottle) / ANGLE_MAX * (param.get_full_throttle_control() - BASE_SPEED)
        if self.x_angle_to_bottle > 0:  # bottle is to the left
            r_speed = BASE_SPEED + speed_increment
            l_speed = param.get_full_throttle_control()
        else:
            r_speed = param.get_full_throttle_control()
            l_speed = BASE_SPEED + speed_increment

        speeds = [l_speed, r_speed, NOSPEED]
        return speeds

    def compute(self):
        # the published information is speeds = [left_wheel, right_wheel, conveyor_belt]
        speeds = [NOSPEED, NOSPEED, NOSPEED]
        if self.time_old_dist is None:
            self.time_old_dist = rospy.get_time()
            passed_time = 0
        else:
            passed_time = rospy.get_time() - self.time_old_dist
            if passed_time > param.get_total_game_time():
                # Game is over
                return speeds # speeds is zero
            else:
                # if we have found a bottle, go to bottle
                if self.bottle_found:
                    speeds = self.calculate_speeds_to_bottle()

                # if front ir detects bottle capture bottle

                # if back ir detects full conv belt

                # if obstacle on the way, avoid


        speeds = self.introduce_speed_slack(speeds)
        return speeds


    def introduce_speed_slack(self, speeds):
        # i had to introduce this slew otherwise arduino rebooted when too sudden changes
        SLEW = param.get_slew_for_speeds_control()
        speeds[0] = np.clip(speeds[0], self.old_speed_x - SLEW, self.old_speed_x + SLEW)
        speeds[1] = np.clip(speeds[1], self.old_speed_y - SLEW, self.old_speed_y + SLEW)
        speeds[2] = np.clip(speeds[2], self.old_speed_theta - SLEW, self.old_speed_theta + SLEW)
        self.old_speed_x = speeds[0] #(speeds[0]>100)*(100 - speeds[0]) + speeds[0]
        self.old_speed_y = speeds[1] #(speeds[1]>100)*(100 - speeds[1]) + speeds[1]  #speeds[1]
        self.old_speed_theta = speeds[2]
        return speeds


if __name__ == '__main__':
    # FIXME what to write here?
    pass