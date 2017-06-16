#!/usr/bin/env python
# license removed for brevity
import rospy
from sticky_robot.msg import bottle_msg
import sys
import time
sys.path.append("/home/odroid/sticky-robot/software/bottle_detection")
import haar_detector as hd

# choose classifier:
classifier = '/home/odroid/sticky-robot/software/bottle_detection/classifiers/final_training_7617.xml'

if __name__ == '__main__':
    try:
        # initialize bottle detector
        bottle_detector = hd.HaarDetector(classifier, 'bottle')
        bottle_detector.init_video()
        pub = rospy.Publisher('bottles', bottle_msg, queue_size=10)
        rospy.init_node('bdetector', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        bt_msg = bottle_msg()
        while not rospy.is_shutdown():
            try:
                bottle_detector.time_counter = bottle_detector.time_counter + 1
                ret = bottle_detector.get_frame(verbose=False, visualisation=False)
                if ret is True:
                    bottle_detector.detect(verbose=False, visualisation=False)
                    bottle_detector.validate_bottle_age(verbose=False)
                    bottle_detector.pixel2angle(bottle_detector.object_list)

                    # # validate list, filter out false positives
                    bottle_detector.filter(verbose=False)

                    bottle_detector.filter_y_angle(verbose=False)

                    # # decide what bottle is pursued
                    final_bottle = bottle_detector.decide()

                    bt_msg.num_bottles = len(bottle_detector.object_list)
                    bt_msg.final_x_angle = final_bottle.get_angle()[0]
                    bt_msg.final_y_angle = final_bottle.get_angle()[1]


                    pub.publish(bt_msg)
                    rate.sleep()
            except Exception as e:
                print(e)
    except rospy.ROSInterruptException:
        pass
