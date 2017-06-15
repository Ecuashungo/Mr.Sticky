import numpy as np

def get_debug_verbose():
    DEBUG = 1  # can be zero or one for debugging perposes
    return DEBUG

def get_covariance_triangulation_kalman():
    TRIANG_SIGMA_XY = 0.03  # [m] this is the estimated standard deviation of x and y of the triangulation
    TRIANG_SIGMA_THETA = 0.08  # [rad] this is the estimated standard deviation of theta of the triangulation
    # Estimated covariance matrix for triangulation measures
    R_TRIANG = np.diag([TRIANG_SIGMA_XY ** 2, TRIANG_SIGMA_XY ** 2, TRIANG_SIGMA_THETA ** 2])

    return R_TRIANG


def get_distance_between_wheels_kalman():
    # FIXME update this value
    DISTANCE_BETWEEN_WHEELS = 0.34  # [m] distance between the wheels
    return DISTANCE_BETWEEN_WHEELS

def get_radius_of_wheel(): # is used in kalman and fsm
    # FIXME update this value
    RADIUS = 0.075  # [m] radius of the wheels
    return RADIUS

def get_covariance_matrix_Q_kalman():
    # This matrix is hard to tune and the core of Kalman
    sigma_model_x = 0.9  # [m] this is th uncertainty of the model
    sigma_model_y = 0.9  # [m]
    sigma_model_theta = 0.3  # [rad]
    return np.diag([sigma_model_x ** 2, sigma_model_y ** 2, sigma_model_theta ** 2])

def get_covariance_odometry_kalman(time_step):
    # this covariance is a function of time, since the longer we have not received a signal, the less we trust the measure
    # FIXME make an intelligent function with time here
    #ODOM_SIGMA_XY = time_step + 0.3  # [m] this is the estimated standard deviation of x and y of the odometry
    #ODOM_SIGMA_THETA = time_step + 0.1  # [rad] this is the estimated standard deviation of theta of the odometry
    ODOM_SIGMA_XY = 0.3  # [m] this is the estimated standard deviation of x and y of the odometry
    ODOM_SIGMA_THETA = 0.3  # [rad] this is the estimated standard deviation of theta of the odometry
    # Estimated covariance matrix for odometry measures
    R_ODOM = np.diag([ODOM_SIGMA_XY ** 2, ODOM_SIGMA_XY ** 2, ODOM_SIGMA_THETA ** 2])
    return R_ODOM




def get_half_throttle_control():
    # this speed is used for turning --> value is within [0, 255]
    HALF_THROTTLE = 125
    return HALF_THROTTLE

def get_full_throttle_control():
    # this speed is used for driving straight --> value is within [0, 255]
    FULL_THROTTLE = 255
    return FULL_THROTTLE

def get_conv_speed_control():
    CONV_SPEED = 125  # FIXME this value can be changed when testing
    # change this maybe to make it turbo release
    return CONV_SPEED

def get_dist_pid_gains_control():
    k_prop = 200
    k_deriv = 0
    k_int = 10
    k_values = [k_prop, k_deriv, k_int]
    return k_values

def get_angle_pid_gains_control():
    k_prop_angle = 450
    k_deriv_angle = 0
    k_int_angle = 10
    k_values_angle = [k_prop_angle, k_deriv_angle, k_int_angle]
    return k_values_angle

def get_slew_for_speeds_control():
    SLEW = 30  # a number between 0 and 255 indicating the slew rate
    # this has actually been used to reduce chance of breaking motors with too abrupt changes
    return SLEW

def build_up_threshold_control():
    THRESHOLD = 10  # this is used to detect if we are trying to hit an obstacle all the time
    return THRESHOLD


def get_obstacle_break_normal_control():
    normal = 30
    return normal

def get_obstacle_break_hard_control():
    hard = 40
    return hard

def get_obstacle_break_very_hard_control():
    very_hard = 50
    return very_hard

def get_obstacle_cst_offset_control():
    cst_offset = 270
    return cst_offset

def get_obstacle_increment_val_control():
    increment_val = 3
    return increment_val

def get_obstacle_decrement_val_control():
    decrement_val = 10
    return decrement_val


def get_total_game_time():
    TIME = 600  # [s] total time of the game in seconds
    return TIME


def get_tolerance_for_immobility():
    tolerance = 0.3  # [m] if we are stuck within this distance, we should get out after some time
    return tolerance

def get_arrival_threshold():  # this is used by fsm and control
    THRESHOLD = 0.2  # [m] if we are within this distance, we consider it as arrival
    return THRESHOLD

def get_arrival_angle_threshold():  # this is used by fsm and control
    ANGLE_THRESHOLD = 0.3  # [rad] if we are within this angle, we consider it as matched
    return ANGLE_THRESHOLD

def get_goal_update_duration_fsm():
    update_time = 180  # [s] time duration allowed until next point should be reached
    return update_time

def get_time_when_last_homing_fsm():
    time_to_drive = 540
    return time_to_drive


def get_turning_angle_increment_fsm():
    TURNING_ANGLE = 0.07 # [rad] the angle deviation that we want to have when on the lookout
    return TURNING_ANGLE

def get_ir_front_threshold_fsm():
    # TODO adapt these values
    #170 - 190 --> normal range
    #190  --> robot end
    THRESHOLD = 200
    return THRESHOLD

def get_ir_left_threshold_fsm():
    # TODO adapt these values
    #0 - 10 --> normal range
    #50 - 70 --> 0.5m
    #80 - 90 --> 0.3m
    THRESHOLD = 90
    return THRESHOLD

def get_ir_center_threshold_fsm():
    # TODO adapt these values
    #0 - 20 --> normal range
    #40 - 50 --> 0.5m
    #90 - 110 --> 0.3m
    THRESHOLD = 90
    return THRESHOLD

def get_ir_right_threshold_fsm():
    # TODO adapt these values
    #0 - 30 --> normal range
    #80 - 90 --> 0.5m
    #110 - 120 --> 0.3m
    THRESHOLD = 120
    return THRESHOLD

def get_ir_back_threshold_fsm():
    # TODO adapt these values
    #140 --> normal range
    #160 - 170  --> robot end
    THRESHOLD = 200
    return THRESHOLD

def get_ir_bottom_threshold_fsm():
    # TODO adapt these values
    #60 --> normal range
    #100 - 150  --> robot end
    THRESHOLD = 300
    return THRESHOLD

def get_loading_time_approach_fsm():
    LOAD_TIME = 5
    return LOAD_TIME

def get_alpha_for_obstacle_avoid_fsm():
    ALPHA = 0.4  # [rad] the angle that is used to rotate the goal
    return ALPHA

def get_factor_for_obstacle_avoid_fsm():
    COEFF = 2  # multiplying factor if obstacle in center
    return COEFF

def get_y_angle_bottle_contact_fsm():
    # FIXME measure this parameter
    Y_LIMIT = 0.5  # [rad] this is the angle where the bottle disappears
    return Y_LIMIT

def get_grabbing_encoder_distance_fsm():
    # FIXME tune this such that it is about 10 cm
    ENCODER_DIST = 0.2  # [m] encoder counter to measure distance
    return ENCODER_DIST


def get_turning_encoder_distance_fsm():
    # FIXME tune this such that it is about 10cm on the conveyor belt
    ENCODER_DIST = 0.1  # [m] encoder counter to measure distance of conveyor belt
    return ENCODER_DIST

def get_ir_filter_coeff_fsm():
    FILTER_COEFF = 0.7  # [-] this is the filter used on the ir, must be between 0 and 1
    return FILTER_COEFF

def get_ir_back_filter_coeff_fsm():
    FILTER_COEFF = 0.95  # [-] this is the filter used on the ir, must be between 0 and 1, should be more conservative
    return FILTER_COEFF

def get_button_hold_time_fsm():
    TIME = 5  # [s] FIXME define a time parameter here
    return TIME

def get_reduction_ratio_motor():  # this is used in kalman and fsm
    REDUCTION = 172
    return REDUCTION

def get_impulsion_encoder():  # this is used in kalman and fsm
    IMPULSIONS = 48
    return IMPULSIONS

def get_distance_of_virtual_bottle_fsm():
    DIST = 5  # [m] distance for next goal
    return DIST

def get_bottle_releasing_time_fsm():
    TIME = 30  # [s] time used to release the bottles
    return TIME