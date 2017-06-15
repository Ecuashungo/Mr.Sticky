import numpy as np
import math
from random import uniform
import sys
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
sys.path.append("C:/Users/Oechslin/sticky-robot/software/parameters/")
import parameters as param
import time
import math
import rospy


# TODO check the grabbing state, and also releasing
# todo tune obstacle avoidance, implement bdetector then implement triangulation


# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging purposes
CV_LOAD = 1
CV_UNLOAD = -1
CV_REST = 0
NO = 0

# Define coordinates for the stage and home
STAGE_X = 5  # [m] x coord where the stage begins
STAGE_Y = 6  # [m] y coord where the stage begins
HOME_XY = 2  # [m] coords for the home base
ARENA_LIMIT_MAX = 7.8  # [m] define the maximum of the arena
ARENA_LIMIT_MIN = 0.2
# FIXME prolong the goal along the direct path and cut it on the border instead of perpendicular shifts


class State_pose:
    def __init__(self, name, pos=[0., 0., 0., 0., 0.]):
        self.name = name
        self.x = pos[0]  # [m]
        self.y = pos[1]  # [m]
        self.theta = pos[2]  # [rad]
        self.conv_direction = pos[3]
        return

    def update(self, pos):
        self.x = float(pos[0])
        self.y = float(pos[1])
        self.theta = float(pos[2])
        self.conv_direction = float(pos[3])
        return np.array([self.x, self.y, self.theta])

    def get_state_pose(self):
        return [self.x, self.y, self.theta, self.conv_direction]

    def get_state_pose_as_vec(self):
        return np.array([[self.x], [self.y], [self.theta], [self.conv_direction]])


class State_machine():
    def __init__(self):
        # Waypoints: zone_1 = center, zone_2 = grass, zone_3 = stones, zone_4 = stage
        self.way_points= {"home": [0.5, 0.5, 0], "zone_1": [4, 4, 0], "zone_2": [4, 1, 0],
                          "zone_3": [1.5, 6.5, 0], "corner": [4.5, 5.5, 0], "before_ramp": [7.75, 5, math.pi/2],
                          "after_ramp": [7.75, 7.5, math.pi/2], "zone_4": [6.5, 7, -math.pi]}
        # FIXME might be easier to save the zone_4 as [corner, before_ramp, after_ramp, zone_4]
        # list of goals that want to be reached
        self.goal_list = []
        self.goal_list.append(self.way_points['zone_1'])  # TODO add goals here
        self.goal_list.append(self.way_points['zone_3'])
        self.goal_list.append(self.way_points['zone_2'])

        # Define intiial state
        self.state = "init"
        self.initial_time = None  # is None until game starts
        self.my_pose = State_pose("state_pose")
        self.my_pose.x = self.way_points['home'][0]
        self.my_pose.y = self.way_points['home'][1]
        self.my_pose.theta = self.way_points['home'][2]

        # define variables updated by bdetector
        self.x_angle_to_bottle = 0
        self.y_angle_to_bottle = 0
        self.bottle_found = False  # boolean value
        # this is the angle permitting to go off the route and collect a bottle
        self.bottle_angle_allowance = math.pi
        self.amount_of_bottle_grab_trials  = 0  # says how many times we have tried to grab a bottle until homing

        self.hunting = False  # indicates if a bottle has been found that we want to get
        self.started_grabbing = False  # indicates whether a grabbing has already been started
        self.started_loading = False  # indicates whether loading has started
        self.releasing = False  # indicates whether we want to release the bottles (only in recycling area)
        self.loading = True  # indicates if we want to load or unload
        self.l_encoder_before_grabbing = 0  # [m] temp value to save encoder distance
        self.r_encoder_before_grabbing = 0
        self.c_encoder_before_turning = 0
        self.i_am_stuck = 0  # counter that indicates if we have spent too much time on the same spot
        self.goal_update_time = None

        # bottle approach variables
        self.loading_time = None
        self.temp_bottle_approach_pos_x = None  # State_pose("temp_bottle")
        self.temp_bottle_approach_pos_y = None

        # initialize readings for ir sensors
        self.ir_front = 0  # is an arbitrary number, somehow related to distance, thresholds are given in param file
        self.ir_left = 0
        self.ir_center = 0
        self.ir_right = 0
        self.ir_back = 0
        self.ir_bottom = 0  # detects if bottle is in front of us

        # initialize bumper readings
        self.bumper_left = False  # says if bumper is pressed
        self.bumper_right = False

        # initialize readings for encoders
        self.l_encoder_dist = 0.0  # [m] distance covered according to encoders
        self.r_encoder_dist = 0.0
        self.c_encoder_dist = 0.0
        return

    # function that checks if the goal corresponds to the actual position given by kalman
    def are_we_there(self, goal):
        THRESHOLD = param.get_arrival_threshold()
        ANGLE_THRESHOLD = param.get_arrival_angle_threshold()
        if (goal.x - self.my_pose.x) < THRESHOLD:
            if (goal.y - self.my_pose.y) < THRESHOLD:
                if (goal.theta - self.my_pose.theta) < ANGLE_THRESHOLD:
                    return True
        else:
            return False
        return

    # this function projects a bottle based on the detected angle this is necessary since it is not known how far the
    # the bottle actually is
    def create_goal_from_bottle(self):
        x_angle = self.x_angle_to_bottle
        theta = self.my_pose.theta

        dist = param.get_distance_of_virtual_bottle_fsm() # [m] distance for next goal
        x_goal = self.my_pose.x + dist * math.cos(x_angle + theta)
        y_goal = self.my_pose.y + dist * math.sin(x_angle + theta)
        theta_goal = self.limit_angle_to_2pi(self.my_pose.theta + x_angle)
        self.goal_list.append([x_goal, y_goal, theta_goal])
        #return self.goal_list[-1]
        return [x_goal, y_goal, theta_goal]

    # this obstacle avoidance is used to smooth out the way and create a new goal
    # there is a second obstacle avoidance implemented in control
    def avoid_obstacles(self, goal):
        # define rotating angle alpha
        alpha = param.get_alpha_for_obstacle_avoid_fsm()  # [rad] the angle that is used to rotate the goal
        coeff = param.get_factor_for_obstacle_avoid_fsm()  # multiplying factor if obstacle in center

        left_blocked = self.ir_left > param.get_ir_left_threshold_fsm()
        center_blocked = self.ir_center > param.get_ir_center_threshold_fsm()
        right_blocked = self.ir_right > param.get_ir_right_threshold_fsm()

        # check IR sensors, check camera
        if DEBUG:
            #print("avoid_obstacles: ", goal)
            pass
        if left_blocked:
            if center_blocked:
                if right_blocked:
                    # all are blocked, complete turn:
                    if DEBUG:
                        print("all are blocked, turn very hard left")
                    new_goal = self.rotate_goal(goal, 4 * coeff * alpha)
                else:
                    # left and center blocked
                    if DEBUG:
                        print("left and center blocked -> turn hard right")
                    new_goal = self.rotate_goal(goal, - coeff * alpha)
            else: # left but not center
                if right_blocked:
                    # left and right blocked
                    if DEBUG:
                        print("left and right blocked -> turn very hard left")
                    new_goal = self.rotate_goal(goal, 4 * coeff * alpha)
                else:
                    # only left blocked
                    if DEBUG:
                        print("left blocked -> turn right")
                    new_goal = self.rotate_goal(goal, - alpha)
        else: # left free
            if center_blocked:
                if right_blocked:
                    # right and center blocked
                    if DEBUG:
                        print("center and right blocked -> turn hard left")
                    new_goal = self.rotate_goal(goal, coeff * alpha)
                else:
                    # only center blocked
                    if DEBUG:
                        print("center blocked -> turn hard left")
                    new_goal = self.rotate_goal(goal, coeff * alpha)
            else:
                if right_blocked:
                    # only right blocked
                    if DEBUG:
                        print("right blocked -> turn left")
                    new_goal = self.rotate_goal(goal, alpha)
                else:
                    # nothing is blocked
                    if DEBUG:
                        print("nothing is blocked -> go straight")
                    new_goal = goal
        return new_goal

    # computes goal based on states and sensory input
    def compute_goal(self):
        # initialize goal as current position for safety stop
        goal = self.my_pose

        if self.initial_time is None:  # we haven't started the game yet
            goal.x = -1
            goal.y = -1
            if DEBUG:
                print("We are in initial state")
            return goal  # end loop if game has not started yet
        elif rospy.get_time() - self.initial_time > param.get_total_game_time():
            if DEBUG:
                print("game is over")
            goal.x = -1  # makes conveyor stop after10 minutes
            goal.y = -1
            return goal
        elif rospy.get_time() - self.initial_time > param.get_time_when_last_homing_fsm():
            self.state = "homing"
            self.bottle_angle_allowance = NO
        elif rospy.get_time() - self.goal_update_time > param.get_goal_update_duration_fsm():
            if self.goal_list is not None:
                del self.goal_list[-1]
            self.goal_update_time = rospy.get_time()
            self.state = "go_to_point"

        if DEBUG:
            #print("compute goal: ", goal)
            #print("time now = ", rospy.get_time(), "line 200")
            pass

        if self.ir_back > param.get_ir_back_threshold_fsm() and self.state is not "init":
            if self.amount_of_bottle_grab_trials > 0:
                # make this only if we have tried to turn conveyor belt at least once
                # bottles have reached back -> go home
                self.state = "homing"
                self.bottle_angle_allowance = NO
                self.amount_of_bottle_grab_trials = 0

        if self.ir_bottom > param.get_ir_bottom_threshold_fsm():
            self.state = "grabbing"
            self.bottle_angle_allowance = NO


        if self.bottle_found and not self.hunting and self.state is not "init":
            if math.fabs(self.x_angle_to_bottle) < self.bottle_angle_allowance:  # bottle within allowance
                if DEBUG:
                    print("I found a bottle and want to go there, in state_machine line 249")
                self.state = "go_to_point"
                new_goal = self.create_goal_from_bottle()
                goal.x = new_goal[0]
                goal.y = new_goal[1]
                goal.theta = new_goal[2]
                print("the appended bottle pos = ", goal, " in state line 255")
                self.hunting = True

        if self.state == 'recovery':
            # The robot is lost and should locate itself
            # FIXME implement this one
            print("The robot is lost and should locate itself")

        elif self.state == 'init':
            # nothing to do here
            goal.x = -1
            goal.y = -1
            print("The robot is in the initializing phase")
            return goal

        elif self.state == 'homing':
            # The robot is full, almost full or time is up
            # in the end, an automatic release procedure is done
            goal = State_pose("state_goal")
            goal.x = self.way_points['home'][0]
            goal.y = self.way_points['home'][1]
            goal.theta = self.way_points['home'][2]

            # check where we are first, to go via waypoints
            if self.robot_is_on_stage():
                # put waypoints before going home
                margin = 0.5  # [m] to not get stuck on the ramp
                if self.my_pose.x > STAGE_X - margin:
                    goal.x = self.way_points['before_ramp'][0]
                    goal.y = self.way_points['before_ramp'][1]
                    goal.theta = self.way_points['before_ramp'][2]
                else:
                    goal.x = self.way_points['after_ramp'][0]
                    goal.y = self.way_points['after_ramp'][1]
                    goal.theta = self.way_points['after_ramp'][2]
                    # FIXME this might be cut off by the corner as soon as it gets to the end of the ramp

            if self.robot_is_at_home():
                # disallow bottles
                self.bottle_angle_allowance = NO
                pass

            if self.are_we_there(goal):
                if self.releasing:
                    # check if time is more than necessary
                    goal = self.my_pose
                    goal.conv_direction = CV_LOAD
                    if rospy.get_time() - self.bottle_release_time > param.get_bottle_releasing_time_fsm():
                        # FIXME move forward to make it easier
                        # change state, no cv speed
                        self.releasing = False
                        self.state = "go_to_point"
                    pass
                else:
                    self.releasing = True
                    goal = self.my_pose
                    goal.conv_direction = CV_UNLOAD
                    self.bottle_release_time = rospy.get_time()
                pass
            print("The robot is full, almost full or time is up")

        elif self.state == 'go_to_point':
            # The robot wants to go to the next point
            if DEBUG:
                print("I am in goto point state")

            if self.goal_list is not None:
                goal = State_pose("state_goal")
                goal.x = self.goal_list[-1][0]
                goal.y = self.goal_list[-1][1]
                goal.theta = self.goal_list[-1][2]

                if DEBUG:
                    print(goal.get_state_pose_as_vec())
                    print("goal = ", goal.x, goal.y, goal.theta, "in state_machine line 328")

                # if we are already there: update state
                if self.are_we_there(goal):
                    # FIXME if stuck too long at same goal, choose another one
                    if self.hunting and self.bottle_found:
                        # if we think we are there but are still hunting
                        new_goal = self.create_goal_from_bottle()
                        goal.x = new_goal[0]
                        goal.y = new_goal[1]
                        goal.theta = new_goal[2]
                        # FIXME del goal_list[-1] check if this is appended so that we should delete it from list
                    else:
                        self.state = "lookout"
                        del self.goal_list[-1]
                        if DEBUG:
                            print("goal list = ", self.goal_list[-1], "in state machine line 344")
                            #self.goal_list.append([0, 0, math.pi])
                            #self.state = "go_to_point"

                if (self.hunting):
                    if self.ir_bottom > param.get_ir_bottom_threshold_fsm(): # FIXME tune this parameter
                        # we are close already, initiate grabbing
                        self.state = "grabbing"
                        self.bottle_angle_allowance = 0
                        if self.goal_list is not None:
                            del self.goal_list[-1]

            else:  # if there are no more points in the list
                self.goal_list.append(self.way_points['zone_1'])
            print("The robot wants to go to the next point")

        elif self.state == 'lookout':
            if DEBUG:
                print("The robot is on the lookout for bottles")
            # self robot is on the lookout for bottles
            goal = self.my_pose
            goal.theta = self.limit_angle_to_2pi(goal.theta + param.get_turning_angle_increment_fsm())

            print("limited angle in lookout state machine line 349 = ", goal.theta)

            self.bottle_angle_allowance = math.pi  # accept a bottle everywhere
            # FIXME do we enter lookout when we go to a bottle, but we don't see it anymroe

        elif self.state == 'grabbing':
            if DEBUG:
                print("The robot is right in front of a bottle and only needs to grab it")

            if self.started_grabbing:
                #print(" I have started grabbing, temp pose is", self.temp_bottle_approach_pos.x, self.temp_bottle_approach_pos.y, " state 373")
                goal.x = self.temp_bottle_approach_pos_x
                goal.y = self.temp_bottle_approach_pos_y

                if not self.loading and self.loading_time is not None:
                    if rospy.get_time() - self.loading_time > param.get_loading_time_approach_fsm() / 2:
                        # we have turned the conveyor belt back at its initial position
                        if DEBUG:
                            print("we have turned the conveyor belt for 5 seconds now")
                        self.started_grabbing = self.loading
                        self.loading = not self.loading
                        self.loading_time = rospy.get_time()
                        self.state = "lookout"
                        goal.conv_direction = CV_REST
                        self.my_pose.conv_direction = CV_REST
                        return goal
                if DEBUG:
                    print("entered the started grabbing line 376 state, left encoder dist =", self.l_encoder_dist, " minus first = ", self.l_encoder_before_grabbing)
                #if True:
                if self.l_encoder_dist - self.l_encoder_before_grabbing > param.get_grabbing_encoder_distance_fsm():
                    #if True:
                    if self.r_encoder_dist - self.r_encoder_before_grabbing > param.get_grabbing_encoder_distance_fsm():
                        # we are at the given distance
                        if DEBUG:
                            print("I entered both if's --> did 20 cm in state line 378")
                        goal = self.my_pose
                        # FIXME stop loading when the second sensor detects something
                        # fixme 2 stop unloading when second sensor detects
                        if self.loading:
                            if DEBUG:
                                print("I started loading and want to turn the cv belt in line 383")
                            goal.conv_direction = CV_LOAD
                            if not self.started_loading:
                                self.c_encoder_before_turning = self.c_encoder_dist
                                self.loading_time = rospy.get_time()
                                self.started_loading = True
                            if self.ir_front > param.get_ir_front_threshold_fsm():
                                if DEBUG:
                                    print("the front cv belt sensor has detected a bottle, stop loading in state line 390")
                                # bottle is on conveyor belt
                                self.started_grabbing = False
                                self.started_loading = False
                                self.loading = True
                                self.state = "go_to_point"
                                self.loading_time = None
                                self.amount_of_bottle_grab_trials += 1
                                print("I switched to goto in line 412")
                                return goal
                        else:
                            goal.conv_direction = CV_UNLOAD


                        if self.started_loading:  # loading maneuver has started, needs to step when threshold is reached
                            if self.c_encoder_dist - self.c_encoder_before_turning > param.get_turning_encoder_distance_fsm():
                                if DEBUG:
                                    print("we have turned enough in line 403")
                                # we have turned too far
                                self.loading = False


                        if not self.loading:
                            if self.c_encoder_dist - self.c_encoder_before_turning < 0:
                                # we have turned the conveyor belt back at its initial position
                                if DEBUG:
                                    print("we have turned the conveyor belt back at its initial position in line 411")
                                self.loading = True
                                self.started_grabbing = False

                        if self.loading_time is not None:
                            if rospy.get_time() - self.loading_time > param.get_loading_time_approach_fsm():
                                # we have turned the conveyor belt back at its initial position
                                if DEBUG:
                                    print("we have turned the conveyor belt for 5 seconds now")
                                self.started_grabbing = self.loading
                                self.loading = not self.loading
                                self.loading_time = rospy.get_time()

            else:
                if DEBUG:
                    print("entered grabbing phase and declared initial state in line 409")
                # save initial pose
                self.l_encoder_before_grabbing = self.l_encoder_dist
                self.r_encoder_before_grabbing = self.r_encoder_dist
                self.started_grabbing = True
                # define goal
                goal = self.my_pose
                goal.x = goal.x + 5 * math.cos(self.my_pose.theta)  # distance of 1 m
                goal.y = goal.y + 5 * math.sin(self.my_pose.theta)
                self.temp_bottle_approach_pos_x = goal.x
                self.temp_bottle_approach_pos_y = goal.y
                if DEBUG:
                    print('Goal: ', goal)
                    print('goal.x: ', goal.x)
                    print('goal.y: ', goal.y)
                #goal.conv_direction = CV_LOAD
                if DEBUG:
                    print("next goal is at ", goal, "in state in line 419")

        else:
            print("Impossible state")
            self.state = "go_to_point"

        if not goal.conv_direction:
            goal = self.avoid_obstacles(goal)

        # FIXME implement this again
        #goal = self.bound_goal_to_arena(goal)
        if DEBUG:
            print("the final new goal is ", goal.get_state_pose_as_vec())


        if goal.x - self.my_pose.x < param.get_tolerance_for_immobility():
            if goal.y - self.my_pose.y < param.get_tolerance_for_immobility():
                self.i_am_stuck += 1
                if DEBUG:
                    print("getting stuck counter is on ", self.i_am_stuck)
                if self.i_am_stuck > 1000:
                    if self.goal_list is not None:
                        del self.goal_list[-1]
                    self.goal_list.append(self.way_points['zone_1'])
                    self.goal_list.append(self.way_points['home'])
                    self.i_am_stuck = 0
                    if DEBUG:
                        print("we are stuck, we want to go home")

        return goal

    def cb_kalman(self, data):
        self.my_pose.x = data.x
        self.my_pose.y = data.y
        self.my_pose.theta = data.theta
        return

    def cb_bdetector(self, data):
        num_bottles = data.num_bottles
        self.bottle_found = num_bottles != 0

        if (num_bottles):
            self.x_angle_to_bottle = data.final_x_angle
            self.y_angle_to_bottle = data.final_y_angle
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

    def cb_bumpers(self, data):
        self.bumper_left = data.bumper_left
        self.bumper_right = data.bumper_right

        # small function that indicates the start of the whole game
        if self.state == "init":
            if self.bumper_left and self.bumper_right:
                if self.initial_time is None:
                    self.initial_time = rospy.get_time()
                    self.goal_update_time = rospy.get_time()
                    if DEBUG:
                        print("init time = ", self.initial_time)
                else:
                    if DEBUG:
                        diff = rospy.get_time() - self.initial_time
                        print("time diff = ", diff)
                    if rospy.get_time() - self.initial_time > param.get_button_hold_time_fsm():  # [s] seconds until robot starts
                        self.initial_time = rospy.get_time()  # FIXME was zero before
                        self.state = "go_to_point"
                        #self.state = "grabbing"
        return

    def cb_encoders(self, data):
        impulsion = param.get_impulsion_encoder()  # impulsions per round
        r_wheel = param.get_radius_of_wheel()
        reduction = param.get_reduction_ratio_motor()
        coeff_conveyor = 0.061  # this parameter has been tested and worked
        if DEBUG:
            #print("encoder callback called in line 439")
            pass
        self.l_encoder_dist = float(data.l_encoder) / impulsion / reduction * 2 * math.pi * r_wheel
        self.r_encoder_dist = float(data.r_encoder) / impulsion / reduction * 2 * math.pi * r_wheel
        self.c_encoder_dist = float(data.c_encoder) / impulsion / reduction * coeff_conveyor
        return

    def rotate_goal(self, goal, angle):
        theta_orientation = self.my_pose.theta
        xdiff = goal.x - self.my_pose.x
        ydiff = goal.y - self.my_pose.y

        new_goal = goal
        new_goal.x = math.cos(angle) * xdiff - math.sin(angle) * ydiff + self.my_pose.x
        new_goal.y = math.sin(angle) * xdiff + math.cos(angle) * ydiff + self.my_pose.y
        return new_goal

    def robot_is_on_stage(self):
        if self.my_pose.x > STAGE_X:
            if self.my_pose.y > STAGE_Y:
                return True
        return False

    def robot_is_at_home(self):
        if self.my_pose.x < HOME_XY:
            if self.my_pose.y < HOME_XY:
                return True
        return False

    def bound_goal_to_arena(self, goal):
        if DEBUG:
            print("goal = ", goal, " in bound goal to arena line 489")
        if goal.x > ARENA_LIMIT_MAX:
            goal.x = ARENA_LIMIT_MAX
        elif goal.x < ARENA_LIMIT_MIN:
            goal.x = ARENA_LIMIT_MIN

        if goal.y > ARENA_LIMIT_MAX:
            goal.y = ARENA_LIMIT_MAX
        elif goal.y < ARENA_LIMIT_MIN:
            goal.y = ARENA_LIMIT_MIN

        # FIXME put also bounds on angle
        return goal

    def limit_angle_to_2pi(self, angle):
        while angle > 2 * math.pi:
            angle -= 2 * math.pi
        while angle < 0:
            angle += 2 * math.pi
        return angle