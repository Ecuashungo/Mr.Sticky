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

# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes
NOSPEED = 0
NOT_VALID = -1

def get_orientation_diff(my_pose, next_pose):
    # angle is positive if we have turned too much counterclockwise
    if get_dist_to_goal(my_pose, next_pose) > param.get_arrival_threshold():
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

    def cb_ir_sensors(self, data):
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
        # the published information is speeds = [left_wheel, right_wheel, conveyor_belt]
        speeds = [NOSPEED, NOSPEED, NOSPEED]
        # FIXME put time limit on state machine rather than here
        if self.time_old_dist is None:
            self.time_old_dist = rospy.get_time()
            passed_time = 0
        else:
            passed_time = rospy.get_time() - self.time_old_dist
        try:
            for i in self.next_pose:
                if i == NOT_VALID:
                    return speeds

            # FIXME: this is weird!
            if self.desired_conv_direction:
                speeds = get_conveyor_speed(self.desired_conv_direction)
            else:
                # are we there?
                dist = get_dist_to_goal(self.my_pose, self.next_pose)
                orientation_diff = get_orientation_diff(self.my_pose, self.next_pose)

                if DEBUG:
                    print("dist = ", dist , "in control line 145")
                    print("orientation diff = ", orientation_diff, "in control line 146")
                if math.fabs(orientation_diff) > param.get_arrival_angle_threshold():
                    # todo instead of sending sign, we could send magnitude
                    if self.build_up > param.build_up_threshold_control():
                        speeds = self.get_speed_for_straight(orientation_diff, dist, passed_time)
                    else:
                        speeds = self.get_speed_for_turn(orientation_diff, passed_time)  # if angle is big: correct angle
                elif dist > param.get_arrival_threshold():
                    speeds = self.get_speed_for_straight(orientation_diff, dist, passed_time)
                    pass
                else:  # no angle and no distance to correct
                    return speeds
                self.old_dist = dist
                self.old_angle = orientation_diff
                self.time_old_dist = rospy.get_time()
                pass

        except Exception as e:
            print(e)
        speeds = self.avoid_obstacles(speeds)

        if DEBUG:
            print("speed before slack, after avoidance = ", speeds, "in control in line 165")
        speeds = self.introduce_speed_slack(speeds)
        if DEBUG:
            print("final speed = ", speeds, "in control line 168")

        self.old_speed_x = speeds[0]
        self.old_speed_y = speeds[1]
        self.old_speed_theta = speeds[2]
        return speeds

    def avoid_obstacles(self, speeds):
        # define rotating angle alpha
        alpha = param.get_alpha_for_obstacle_avoid_fsm()  # [rad] the angle that is used to rotate the goal
        coeff = param.get_factor_for_obstacle_avoid_fsm()  # multiplying factor if obstacle in center

        left_blocked = self.ir_left > param.get_ir_left_threshold_fsm()
        center_blocked = self.ir_center > param.get_ir_center_threshold_fsm()
        right_blocked = self.ir_right > param.get_ir_right_threshold_fsm()

        # check IR sensors, check camera
        normal = param.get_obstacle_break_normal_control()
        hard = param.get_obstacle_break_hard_control()
        very_hard = param.get_obstacle_break_very_hard_control()
        cst_offset = param.get_obstacle_cst_offset_control()
        increment_val = param.get_obstacle_increment_val_control()
        decrement_val = param.get_obstacle_decrement_val_control()

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
                        speeds[0] += 20  # values have been found empirically
                        speeds[1] += 20

                        slacked = self.introduce_speed_slack(speeds)
                        speeds[0] = slacked[0]
                        speeds[1] = slacked[1]
                        self.build_up -= decrement_val

                    if DEBUG:
                        print("nothing is blocked -> continue")
                        print("slacked speed in control line 255 = ", speeds)
                        pass
        self.build_up += increment_val
        if DEBUG:
            print("build up is: ", self.build_up, "in control line 259")
        return limit_full_throttle(speeds)


    def introduce_speed_slack(self, speeds):
        # i had to introduce this slew otherwise arduino rebooted when too sudden changes
        # and motors broke down
        SLEW = param.get_slew_for_speeds_control()
        speeds[0] = np.clip(speeds[0], self.old_speed_x - SLEW - SLEW * (speeds[0] > 0), self.old_speed_x + SLEW + SLEW * (speeds[0] < 0))
        speeds[1] = np.clip(speeds[1], self.old_speed_y - SLEW - SLEW * (speeds[1] > 0), self.old_speed_y + SLEW + SLEW * (speeds[1] < 0))
        speeds[2] = np.clip(speeds[2], self.old_speed_theta - SLEW, self.old_speed_theta + SLEW)

        return limit_full_throttle(speeds)

    def get_speed_for_straight(self, orientation_diff, dist, passed_time):
        if DEBUG:
            print("I want to go straight in control in line 275")
        gains = param.get_dist_pid_gains_control()
        # pid gains, but for now mainly p and a bit of an i is used
        k_prop = gains[0]
        k_deriv = gains[1]
        k_int = gains[2]
        self.total_arrival_error += dist * passed_time

        forward_speed = k_prop * dist + k_deriv * (dist - self.old_dist) / passed_time
        forward_speed += k_int * self.total_arrival_error
        if DEBUG:
            print("dist = ", dist, "line 288 in control")
            print("forward speed = ", forward_speed, "line 289 in control")

        # TODO implement angle depending difference for lw_speed and rw_speed
        lw_speed = forward_speed
        rw_speed = forward_speed
        cv_speed = NOSPEED

        speeds = [lw_speed, rw_speed, cv_speed]
        if DEBUG:
            print("speed = ", speeds, "in driving straight fctn control line 296")
        return limit_full_throttle(speeds)

    def get_speed_for_turn(self, angle, passed_time):
        # direction is positive if we have turned too much counterclockwise
        print("I want to turn in control in line 301")
        gains = param.get_angle_pid_gains_control()
        k_prop = gains[0]
        k_deriv = gains[1]
        k_int = gains[2]

        self.total_arrival_angle_error += angle * passed_time
        speed_value = k_prop * math.fabs(angle) + k_deriv * math.fabs(angle - self.old_angle) / passed_time
        speed_value += k_int * math.fabs(self.total_arrival_angle_error)


        # this has been implemented to avoid motor breaking, turns with a radius, not practical
        # if angle > 0:  # want to turn to the right
        #     lw_speed = speed_value
        #     rw_speed = lw_speed - 50
        # else:  # want to turn to the left
        #     rw_speed = speed_value
        #     lw_speed = rw_speed - 50
        lw_speed = np.sign(angle) * speed_value
        rw_speed = - lw_speed
        cv_speed = NOSPEED
        speeds = [lw_speed, rw_speed, cv_speed]
        if DEBUG:
            print("angle  error = ", angle)
            print("k_prop = ", k_prop)
            print("speed magnitude = ", speed_value)
            print("speed = ", speeds, "in turning fctn control line 327")
        return limit_full_throttle(speeds)


if __name__ == '__main__':
    pass



