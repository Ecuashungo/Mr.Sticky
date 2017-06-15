#!/usr/bin/env python
# license removed for brevity

import sys
#sys.path.append("/home/odroid/sticky-robot/software/state_machine")
#import robot_structure as rs
sys.path.append("/home/odroid/sticky-robot/software/control")
import control as ct
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
import parameters as param
import rospy

from sticky_robot.msg import pos_msg
from std_msgs.msg import Int16MultiArray
from sticky_robot.msg import ir_array_msg
# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes
NOSPEED = 0

if __name__ == '__main__':
    # FIXME we could implement rospy.time here to stop (zero speed at 10 mintues)
    ctrl = ct.Control()

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber("state_machine", pos_msg, ctrl.cb_state_machine)
    rospy.Subscriber("kalman", pos_msg, ctrl.cb_kalman)
    rospy.Subscriber("IR", ir_array_msg, ctrl.cb_ir_sensors)

    # todo publish constantly the new control (if not available, publish old until 10s passed), then publish 0 control
    pub = rospy.Publisher('control', Int16MultiArray , queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    ctrl.game_start = rospy.get_time()
    try:
        while not rospy.is_shutdown():

            my_ctrl_msg = Int16MultiArray()
            ctrl_msg_data = ctrl.compute()
            my_ctrl_msg.data = ctrl_msg_data

            if DEBUG:
                print (my_ctrl_msg)

            pub.publish(my_ctrl_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()