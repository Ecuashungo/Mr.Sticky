import numpy as np
import math
import sys
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
import parameters as param
import rospy  # mainly for get_time

# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes
THOUSAND = 1000  # just a big number

class KalmanFilter:
    def __init__(self):
        # H is the transformation matrix of the measurement
        self.H = np.identity(3)
        # Q is the uncertainty of the model

        self.Q_model_cov = param.get_covariance_matrix_Q_kalman()

        # P is the prediction estimate covariance
        self.P_pred_est_cov = 100 * np.identity(3)  # is only zero if position is known with absolute certainty

        # Define time step
        self.dT = None  # This has been updated with the real timestep

        # Control variables
        self.x_estimate = Pose("estimation")
        self.u_control = np.array([[0], [0]])  # [rad/s] set initial speed and omega
        self.old_l_encoder_dist = 0
        self.old_r_encoder_dist = 0
        self.last_enc_update_time = None
        self.last_propagate_time = None
        return

    def estimate(self, measurement, covariance):
        # Calculate next step
        x_pred = self.propagate()

        # Calculate gain
        F = self.get_jacobian(x_pred[2][0])
        P_intermediary = self.estimate_covariance(F)

        y_resid = measurement - np.dot(self.H, x_pred)  # This is the measurement residual
        # This wraps the angle difference to 2 pi
        if y_resid[2][0] > math.pi:
            y_resid[2][0] = y_resid[2][0] - 2 * math.pi
        S = np.dot(np.dot(self.H, P_intermediary), self.H.T) + covariance  # This is the residual covariance

        try:  # exception when matrix not invertible
            K_gain = np.dot(np.dot(P_intermediary, self.H.T), np.linalg.inv(S))  # This is the Kalman gain
        except Exception as e:
            K_gain = np.diag([0, 0, 0])
            print(e)

        # Update the new state and estimate covariance
        new_x_estimate = limit_angle_2pi(x_pred + np.dot(K_gain, y_resid))
        self.x_estimate.update(new_x_estimate)
        self.set_P_matrix(np.dot((np.identity(3) - np.dot(K_gain, self.H)), P_intermediary))

        return

    def propagate(self):
        if self.last_propagate_time is not None:
            passed_time = rospy.get_time() - self.last_propagate_time
        else:
            passed_time = THOUSAND
            self.last_propagate_time = rospy.get_time()
        self.last_propagate_time = rospy.get_time()
        x_new = RK4(self.x_estimate.get_pose_as_vec(), self.u_control, passed_time)
        print("I propagated x in kalman line 70, x = ", x_new)
        return x_new

    def get_jacobian(self, theta):
        # F is the Jacobian of the robot
        speed = self.u_control[0][0]
        jacobian = np.array([[0, 0, -speed * math.sin(theta)],
                             [0, 0, speed * math.cos(theta)],
                             [0, 0, 0]])
        return jacobian

    def estimate_covariance(self, jacobian):
        P_old = self.P_pred_est_cov
        Q = self.Q_model_cov
        F = np.identity(3) + self.dT * jacobian
        P_new = np.dot(np.dot(F, P_old), F.T) + Q
        return P_new

    def set_P_matrix(self, P):
        self.P_pred_est_cov = P
        return

    def set_u_control(self, u):
        self.u_control = u
        return

    def set_right_odom_speed(self, rspeed):
        self.right_wheel_speed = rspeed
        self.right_speed_update_time = rospy.get_time()
        return

    def get_right_odom_speed(self):
        return self.right_wheel_speed

    def set_left_odom_speed(self, lspeed):
        self.left_wheel_speed = lspeed
        self.left_speed_update_time = rospy.get_time()
        return

    def get_left_odom_speed(self):
        return self.left_wheel_speed


    def cb_control(self, data):
        if DEBUG:
            print("cb_control has been called in kalman line 115")
        # data = [right_wheel_speed, left_wheel_speed, conveyor_belt_speed] is between [0..255]
        v_right = data.data[0] / 255 * 0.5 # Full speed is about 0.5 m / s (empirically)
        v_left = data.data[1] / 255 * 0.5

        forward_speed = (v_right + v_left) / 2
        angular_speed = (v_right - v_left) / param.get_distance_between_wheels_kalman()
        self.set_u_control(np.array([[forward_speed], [angular_speed]]))
        return


    def cb_triangulation(self, data):
        if DEBUG:
            print("cb_triangulation has been called in kalman line 128")
        x_pos = data.x
        y_pos = data.y
        theta = data.theta

        measurement = np.array([[x_pos], [y_pos], [theta]])
        covariance = param.get_covariance_triangulation_kalman()
        self.estimate(measurement, covariance)
        return

    def cb_encoders(self, data):
        if DEBUG:
            print("cb_encoders has been called in kalman line 140")
        impulsion = param.get_impulsion_encoder()  # impulsions per round
        r_wheel = param.get_radius_of_wheel()
        reduction = param.get_reduction_ratio_motor()
        if DEBUG:
            print("raw encoder measures = ", data.l_encoder, data.r_encoder)
            pass

        l_enc_dist = float(data.l_encoder) / impulsion / reduction * 2 * math.pi * r_wheel
        r_enc_dist = float(data.r_encoder) / impulsion / reduction * 2 * math.pi * r_wheel

        delta_l_enc = l_enc_dist - self.old_l_encoder_dist
        delta_r_enc = r_enc_dist - self.old_r_encoder_dist
        self.old_l_encoder_dist = l_enc_dist
        self.old_r_encoder_dist = r_enc_dist

        if self.last_enc_update_time is not None:
            time_step = rospy.get_time() - self.last_enc_update_time
        else:
            time_step = THOUSAND   # just a huge number
        self.last_enc_update_time = rospy.get_time()

        delta_l = delta_l_enc
        delta_r = delta_r_enc

        new_x = self.x_estimate.x + (delta_l + delta_r) / 2 * math.cos(self.x_estimate.theta)
        new_y = self.x_estimate.y + (delta_l + delta_r) / 2 * math.sin(self.x_estimate.theta)

        if DEBUG:
            print("new x and y = ", new_x, new_y, "in line 169 kalman")
            pass
        new_theta = self.x_estimate.theta + math.atan2(delta_r - delta_l, param.get_distance_between_wheels_kalman())
        measured_pose_odom = np.array([[new_x], [new_y], [new_theta]])

        if DEBUG:
            print("measured pose odometry: ", measured_pose_odom, " in kalman line 175")
        # apply filter on odometry
        covariance = param.get_covariance_odometry_kalman(time_step)
        self.estimate(measured_pose_odom, covariance)
        return


def RK4(x, u, dT):
    # Runge Kutta method
    k1 = dynamics_f(x, u)
    k2 = dynamics_f(x + 0.5 * dT * k1, u)
    k3 = dynamics_f(x + 0.5 * dT * k2, u)
    k4 = dynamics_f(x + dT * k3, u)
    dx = dT / 6 * (k1 + 2*k2 + 2*k3 + k4)
    return x + dx


def dynamics_f(x, u):
    # This is the dynamics of the system
    # u is [[v], [omega]]
    theta = x[2][0]
    rot_matrix = np.array([[np.cos(theta), 0],
                           [np.sin(theta), 0],
                           [0, 1]])
    xdot = np.dot(rot_matrix, u)
    return xdot

# This works better than np.wrap from python because of how we defined the angles
def limit_angle_2pi(x_vec):
    angle = x_vec[2]
    while angle >= 2 * np.pi:
        angle -= 2 * np.pi
    while angle < 0:
        angle += 2 * np.pi
    x_vec[2] = angle
    return x_vec

class Pose:
    def __init__(self, name, pos=[0., 0., 0.]):
        self.name = name
        self.x = pos[0]  # [m]
        self.y = pos[1]  # [m]
        self.theta = pos[2]  # [rad]
        return

    def update(self, pos):
        self.x = float(pos[0])
        self.y = float(pos[1])
        self.theta = float(pos[2])
        return np.array([self.x, self.y, self.theta])

    def get_pose(self):
        return [self.x, self.y, self.theta]

    def get_pose_as_vec(self):
        return np.array([[self.x], [self.y], [self.theta]])



if __name__ == '__main__':
    pass

