#!/usr/bin/env python
# license removed for brevity

import sys
sys.path.append("/home/odroid/sticky-robot/software/navigation/kalman_filter/")
import kalman_filter as kf
import rospy
from sticky_robot.msg import pos_msg
from sticky_robot.msg import pos_360_msg
from sticky_robot.msg import enc_array_msg
from std_msgs.msg import Int16MultiArray

if __name__ == '__main__':
    my_kf = kf.KalmanFilter()

    rospy.init_node('kalman', anonymous=True)
    rospy.Subscriber("encoders", enc_array_msg, my_kf.cb_encoders)
    rospy.Subscriber("control", Int16MultiArray, my_kf.cb_control)
    rospy.Subscriber("pos_360", pos_360_msg, my_kf.cb_triangulation)

    # publish constantly the estimated state
    pub = rospy.Publisher('kalman', pos_msg, queue_size=1)
    rate = rospy.Rate(10)  # 10hz
    try:
        while not rospy.is_shutdown():
            state_estimate = pos_msg()
            data = my_kf.x_estimate.get_pose()
            state_estimate.x = data[0]
            state_estimate.y = data[1]
            state_estimate.theta = data[2]
            state_estimate.header.stamp = rospy.Time.now()

            # publish the state estimate
            rospy.loginfo(state_estimate)

            pub.publish(state_estimate)
            rate.sleep()


    except rospy.ROSInterruptException:
        pass

    rospy.spin()