#!/usr/bin/env python
# license removed for brevity

import sys
sys.path.append("/home/odroid/sticky-robot/software/state_machine/")
import finite_state_machine as fsm
import rospy
from std_msgs.msg import Float32MultiArray
from sticky_robot.msg import pos_msg
from sticky_robot.msg import bottle_msg
from sticky_robot.msg import ir_array_msg
from sticky_robot.msg import bumper_msg
from sticky_robot.msg import enc_array_msg
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
import parameters as param

# Define parameters
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes


if __name__ == '__main__':
    state_machine = fsm.State_machine()

    rospy.init_node('state_machine', anonymous=True)
    rospy.Subscriber("bottles", bottle_msg, state_machine.cb_bdetector)
    rospy.Subscriber("kalman", pos_msg, state_machine.cb_kalman)
    rospy.Subscriber("IR", ir_array_msg, state_machine.cb_ir_sensors)
    rospy.Subscriber("bumpers", bumper_msg, state_machine.cb_bumpers)
    rospy.Subscriber("encoders", enc_array_msg, state_machine.cb_encoders)

    # publish constantly the estimated state
    pub = rospy.Publisher('state_machine', pos_msg, queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    # For debugging purposes
    if DEBUG:
        pub_debug_ir = rospy.Publisher('filtered_ir', ir_array_msg, queue_size=1)
        pub_debug_enc = rospy.Publisher('converted_encs', Float32MultiArray, queue_size=1)

    try:
        while not rospy.is_shutdown():
            goal = state_machine.compute_goal()

            goal_msg = pos_msg()
            goal_msg.x = goal.x
            goal_msg.y = goal.y
            goal_msg.theta = goal.theta
            goal_msg.conv_direction = goal.conv_direction
            goal_msg.header.stamp = rospy.Time.now()

            pub.publish(goal_msg)

            if DEBUG:
                publish_infrared_readings = 1
                if publish_infrared_readings:
                    filtered_ir = ir_array_msg()
                    filtered_ir.ir_front = state_machine.ir_front
                    filtered_ir.ir_left = state_machine.ir_left
                    filtered_ir.ir_center = state_machine.ir_center
                    filtered_ir.ir_right = state_machine.ir_right
                    filtered_ir.ir_back = state_machine.ir_back
                    filtered_ir.ir_bottom = state_machine.ir_bottom
                    pub_debug_ir.publish(filtered_ir)

                publish_enc_readings = 1
                if publish_enc_readings:
                    converted_encoders = Float32MultiArray()
                    converted_encoders.data.append(state_machine.l_encoder_dist)
                    converted_encoders.data.append(state_machine.r_encoder_dist)
                    converted_encoders.data.append(state_machine.c_encoder_dist)
                    pub_debug_enc.publish(converted_encoders)



            rate.sleep()


    except rospy.ROSInterruptException:
        pass

    rospy.spin()