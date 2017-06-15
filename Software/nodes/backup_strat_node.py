#!/usr/bin/env python
# license removed for brevity

import numpy as np
import math
from random import uniform
import sys
sys.path.append("/home/odroid/sticky-robot/software/control")
import backup_strat as strat
sys.path.append("/home/odroid/sticky-robot/software/parameters/")
import parameters as param
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int16
from sticky_robot.msg import pos_msg
from std_msgs.msg import Int16MultiArray
from sticky_robot.msg import ir_array_msg
from sticky_robot.msg import bottle_msg
#from sticky_robot.msg import ctrl_msg

# Define constants
DEBUG = param.get_debug_verbose() # can be zero or one for debugging perposes
NOSPEED = 0

if __name__ == '__main__':
    # FIXME we could implement rospy.time here to stop (zero speed at 10 mintues)
    my_strat = strat.Strategy()

    rospy.init_node('control', anonymous=True)

#    rospy.Subscriber("sensors", Int16MultiArray, ctrl.cb_sensors)  # need to define int array for arduino
    rospy.Subscriber("bdetector", bottle_msg, my_strat.cb_bdetector)
    rospy.Subscriber("IR", ir_array_msg, my_strat.cb_ir_sensors)

    # publish constantly the new control (if not available, publish old until 10s passed), then publish 0 control
    pub = rospy.Publisher('control', Int16MultiArray , queue_size=1)  # FIXME what should queue size be?
    rate = rospy.Rate(10)  # 10hz

    try:
        while not rospy.is_shutdown():

            my_ctrl_msg = Int16MultiArray()
            ctrl_msg_data = strat.compute()
            #print("afterwards", ctrl_msg_data[0])
            my_ctrl_msg.data = ctrl_msg_data

            if DEBUG:
                print (my_ctrl_msg)


            # publish the speeds
            #rospy.loginfo(Int16MultiArray, convertedmsg)
            pub.publish(my_ctrl_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    # TODO check if spin is endlessloop --> it kinda does
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()