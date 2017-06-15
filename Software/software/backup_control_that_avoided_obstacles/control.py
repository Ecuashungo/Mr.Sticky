#!/usr/bin/env python
# license removed for brevity

import numpy as np
import math
import sys
#sys.path.append("C:/Users/Oechslin/sticky-robot/software/state_machine/")
#sys.path.append("/home/odroid/sticky-robot/software/state_machine/")
#import robot_structure as rs
import time  # FIXME probably not used anymore
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
import parameters as param
import rospy
import numpy as np

# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes
NOSPEED = 0
NOT_VALID = -1

def get_orientation_diff(my_pose, next_pose):
    # angle is positive if we have turned too much counterclockwise
    if get_dist_to_goal(my_pose, next_pose) > 0.05:
        x_diff = next_pose[0] - my_pose[0]
        y_diff = next_pose[1] - my_pose[1]
        theta_desired = math.atan2(y_diff, x_diff)
        orientation_diff = my_pose[2] - theta_desired
    else:  # if we want to correct the angle on the same place
        orientation_diff = next_pose[2] - my_pose[2]
    return orientation_diff


def get_dist_to_goal(my_pose, next_pose):
    x_diff = my_pose[0] - next_pose[0]
    y_diff = my_pose[1] - next_pose[1]

    distance = np.sqrt(x_diff ** 2 + y_diff ** 2)
    return distance


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
        self.my_pose = [0, 0, 0]
        self.next_pose = [0, 0, 0]
        self.desired_conv_direction = NOSPEED

        self.game_start = None

        self.old_dist = 0  # [m] last distance to goal for derivative effect of speed
        self.time_old_dist = None  # [s] seconds that have passed since this value has last been updated

        self.old_speed_x = NOSPEED
        self.old_speed_y = NOSPEED
        self.old_speed_theta = NOSPEED

        self.total_arrival_error = 0  # error for the integrator term of the PID controller in distance
        self.total_arrival_angle_error = 0  # error on the integrator term of the PID controller in angle
        self.old_angle = 0

        # FIXME trial to implement obstacle avoidance 2 control
        # initialize readings for ir sensors
        self.ir_front = 0  # is an arbitrary number, somehow related to distance, thresholds are given in param file
        self.ir_left = 0
        self.ir_center = 0
        self.ir_right = 0
        self.ir_back = 0
        self.ir_bottom = 0  # detects if bottle is in front of us

        # obstacle avoidance variable to move again after waiting
        self.build_up = 0
        return

    def cb_state_machine(self, data):
        x_state = data.x
        y_state = data.y
        theta_state = data.theta
        self.next_pose = [x_state, y_state, theta_state]
        self.desired_conv_direction = data.conv_direction
        return

    def cb_kalman(self, data):
        x_kalman = data.x
        y_kalman = data.y
        theta_kalman = data.theta
        self.my_pose = [x_kalman, y_kalman, theta_kalman]
        return

    def cb_ir_sensors(self, data):  # FIXME trial to implement obstacle avoidance 2 control
        filter_coeff = param.get_ir_filter_coeff_fsm()  # [-] this is the filter used on the ir, must be between 0 and 1
        filter_coeff_back = param.get_ir_back_filter_coeff_fsm()  # [-] this is the filter used on the ir, must be between 0 and 1
        self.ir_front = filter_coeff * self.ir_front + (1 - filter_coeff) * data.ir_front
        self.ir_left = filter_coeff * self.ir_left + (1 - filter_coeff) * data.ir_left
        self.ir_center = filter_coeff * self.ir_center + (1 - filter_coeff) * data.ir_center
        self.ir_right = filter_coeff * self.ir_right + (1 - filter_coeff) * data.ir_right
        self.ir_bottom = filter_coeff * self.ir_bottom + (1 - filter_coeff) * data.ir_bottom
        self.ir_back = filter_coeff_back * self.ir_back + (1 - filter_coeff_back) * data.ir_back
        return


    def compute(self):
        # the published information is speeds = [[left_wheel], [right_wheel], [conveyor_belt]]
        speeds = [NOSPEED, NOSPEED, NOSPEED]
        if self.time_old_dist is None:
            self.time_old_dist = rospy.get_time()
            passed_time = 0
        else:
            passed_time = rospy.get_time() - self.time_old_dist
        try:
            for i in self.next_pose:
                if i == NOT_VALID:
                    return speeds
            if self.desired_conv_direction:
                speeds = get_conveyor_speed(self.desired_conv_direction)
            else:
                # are we there?
                dist = get_dist_to_goal(self.my_pose, self.next_pose)
                orientation_diff = get_orientation_diff(self.my_pose, self.next_pose)

                if DEBUG:
                    print("dist = ", dist , "in control line 120")
                    print("orientation diff = ", orientation_diff, "in control line 121")
                if math.fabs(orientation_diff) > param.get_arrival_angle_threshold():
                    # todo instead of sending sign, we could send magnitude
                    if self.build_up > param.build_up_threshold_control():
                        speeds = self.get_speed_for_straight(orientation_diff, dist, passed_time)
                    else:
                        speeds = self.get_speed_for_turn(orientation_diff, passed_time)  # if angle is big: correct angle
                elif dist > param.get_arrival_threshold():
                    speeds = self.get_speed_for_straight(orientation_diff, dist, passed_time)
                    pass
                self.old_dist = dist
                self.old_angle = orientation_diff
                self.time_old_dist = rospy.get_time()
                pass

        except Exception as e:
            print(e)

        speeds = self.introduce_speed_slack(speeds)
        if DEBUG:
            print("speed before avoidance = ", speeds, "in control in line 162")
        speeds = self.avoid_obstacles(speeds)
        if DEBUG:
            print("final speed = ", speeds, "in control line 137")
            # minN = -50
            # maxN = 50
            # speeds[0] = sorted([minN, speeds[0], maxN])[1]
            # speeds[1] = sorted([minN, speeds[1], maxN])[1]
            # speeds[2] = sorted([minN, speeds[2], maxN])[1]
        return speeds

    def avoid_obstacles(self, speeds):
        # define rotating angle alpha
        alpha = param.get_alpha_for_obstacle_avoid_fsm()  # [rad] the angle that is used to rotate the goal
        coeff = param.get_factor_for_obstacle_avoid_fsm()  # multiplying factor if obstacle in center

        left_blocked = self.ir_left > param.get_ir_left_threshold_fsm()
        center_blocked = self.ir_center > param.get_ir_center_threshold_fsm()
        right_blocked = self.ir_right > param.get_ir_right_threshold_fsm()

        # check IR sensors, check camera
        normal = 30
        hard = 40
        very_hard = 50
        cst_offset = 270
        threshold_build_up = 10  # this is used to detect if we are trying to hit an obstacle all the time


        if left_blocked:
            if center_blocked:
                if right_blocked:
                    # all are blocked, complete turn:
                    if DEBUG:
                        print("all are blocked, turn very hard left")
                    speeds[0] -= very_hard + cst_offset + self.build_up
                    speeds[1] += very_hard + cst_offset + self.build_up
                else:
                    # left and center blocked
                    if DEBUG:
                        print("left and center blocked -> turn hard right")
                    speeds[0] += hard + cst_offset + self.build_up
                    speeds[1] -= hard + cst_offset + self.build_up
            else: # left but not center
                if right_blocked:
                    # left and right blocked
                    if DEBUG:
                        print("left and right blocked -> turn very hard left")
                    speeds[0] -= very_hard + cst_offset + self.build_up
                    speeds[1] += very_hard + cst_offset + self.build_up
                else:
                    # only left blocked
                    if DEBUG:
                        print("left blocked -> turn right")
                    speeds[0] += normal + cst_offset + self.build_up
                    speeds[1] -= normal + cst_offset + self.build_up
        else: # left free
            if center_blocked:
                if right_blocked:
                    # right and center blocked
                    if DEBUG:
                        print("center and right blocked -> turn hard left")
                    speeds[0] -= hard + cst_offset + self.build_up
                    speeds[1] += hard + cst_offset + self.build_up
                else:
                    # only center blocked
                    if DEBUG:
                        print("center blocked -> turn hard left")
                    speeds[0] -= hard + cst_offset + self.build_up
                    speeds[1] += hard + cst_offset + self.build_up
            else:
                if right_blocked:
                    # only right blocked
                    if DEBUG:
                        print("right blocked -> turn left")
                    speeds[0] -= normal + cst_offset + self.build_up
                    speeds[1] += normal + cst_offset + self.build_up
                else:
                    # nothing is blocked
                    if self.build_up > param.build_up_threshold_control():
                        if DEBUG:
                            print("build up was high in control line 245, ", self.build_up)
                            speeds[0] += 20
                            speeds[1] += 20
                    self.build_up = 0

                    if DEBUG:
                        print("nothing is blocked -> go straight")
                        pass
        self.build_up += 3
        if DEBUG:
            print("build up is: ", self.build_up, "in control line 252")
        return limit_full_throttle(speeds)


    def introduce_speed_slack(self, speeds):
        # i had to introduce this slew otherwise arduino rebooted when too sudden changes
        SLEW = param.get_slew_for_speeds_control()
        speeds[0] = np.clip(speeds[0], self.old_speed_x - SLEW - SLEW * (speeds[0] > 0), self.old_speed_x + SLEW + SLEW * (speeds[0] < 0))
        speeds[1] = np.clip(speeds[1], self.old_speed_y - SLEW - SLEW * (speeds[1] > 0), self.old_speed_y + SLEW + SLEW * (speeds[1] < 0))
        speeds[2] = np.clip(speeds[2], self.old_speed_theta - SLEW, self.old_speed_theta + SLEW)
        self.old_speed_x = speeds[0]  #(speeds[0]>100)*(100 - speeds[0]) + speeds[0]
        self.old_speed_y = speeds[1]  #(speeds[1]>100)*(100 - speeds[1]) + speeds[1]  #speeds[1]
        self.old_speed_theta = speeds[2]
        return limit_full_throttle(speeds)

    def get_speed_for_straight(self, orientation_diff, dist, passed_time):
        print("I want to go straight in control in line 160")
        gains = param.get_dist_pid_gains_control()
        k_prop = gains[0]
        k_deriv = gains[1]
        k_int = gains[2]
        self.total_arrival_error += dist * passed_time

        # TODO implement derivative and integral effect and tune parameter
        forward_speed = k_prop * dist + k_deriv * (dist - self.old_dist) / passed_time
        forward_speed += k_int * self.total_arrival_error
        # print(passed_time)
        if DEBUG:
            print("dist = ", dist, "line 56 in control")
            print("forward speed = ", forward_speed, "line 57 in control")

        # TODO implement angle depending difference for lw_speed and rw_speed
        lw_speed = forward_speed
        rw_speed = forward_speed
        cv_speed = NOSPEED

        speeds = [lw_speed, rw_speed, cv_speed]
        # FIXME convert to 16bit integer array first
        if DEBUG:
            print("speed = ", speeds, "in driving straight fctn control line 188")
        return limit_full_throttle(speeds)

    def get_speed_for_turn(self, angle, passed_time):
        # direction is positive if we have turned too much counterclockwise
        print("I want to turn in control in line 189")
        gains = param.get_angle_pid_gains_control()
        k_prop = gains[0]
        k_deriv = gains[1]
        k_int = gains[2]

        self.total_arrival_angle_error += angle * passed_time
        speed_value = k_prop * math.fabs(angle) + k_deriv * math.fabs(angle - self.old_angle) / passed_time
        speed_value += k_int * math.fabs(self.total_arrival_angle_error)


        # this has been implemented to avoid motor breaking, turns with a radius
        # if angle > 0:  # want to turn to the right
        #     lw_speed = speed_value
        #     rw_speed = lw_speed - 50
        # else:  # want to turn to the left
        #     rw_speed = speed_value
        #     lw_speed = rw_speed - 50
        # this was the old method of turning on the spot
        lw_speed = np.sign(angle) * speed_value
        rw_speed = - lw_speed
        cv_speed = NOSPEED
        speeds = [lw_speed, rw_speed, cv_speed]
        # FIXME convert to 16bit integer array first
        if DEBUG:
            print("angle  error = ", angle)
            print("k_prop = ", k_prop)
            print("speed magnitude = ", speed_value)
            print("speed = ", speeds, "in turning fctn control line 207")
        return limit_full_throttle(speeds)


if __name__ == '__main__':
    # FIXME what to write here?
    pass


def test_run():
    # FIXME this is not used anymore
    my_control = Control()
    print(my_control.compute())
    goal = [1, 1, 0.25, 0]
    my_control.cb_state_machine(goal)
    print(my_control.compute())
    my_control.cb_kalman([0, 0, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.1, 0.1, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.2, 0.2, 0.68])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.3, 0.3, 0.67])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.4, 0.4, 0.74])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.5, 0.5, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.6, 0.6, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.7, 0.7, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.8, 0.8, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    my_control.cb_kalman([0.9, 0.9, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
    goal = [1, 1, 0.25, 1]
    my_control.cb_state_machine(goal)
    my_control.cb_kalman([0.9, 0.9, 0.7])
    time.sleep(0.1)
    print(my_control.compute())
