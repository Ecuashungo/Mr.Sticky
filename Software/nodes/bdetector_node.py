#!/usr/bin/env python
# license removed for brevity
import rospy
from sticky_robot.msg import bottle_msg
import sys
import time
sys.path.append("/home/odroid/sticky-robot/software/bottle_detection")
import haar_detector as hd

# choose classifier:
classifier = '/home/odroid/sticky-robot/software/bottle_detection/classifiers/haarcascade_frontalface_default.xml'

if __name__ == '__main__':
    try:
        start_time = time.time()
        # initialize bottle detector
        bottle_detector = hd.HaarDetector(classifier, 'bottle')
        bottle_detector.visualize(False)
        bottle_detector.set_timer(True)
        pub = rospy.Publisher('bottles', bottle_msg, queue_size=10)
        rospy.init_node('bdetector', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        print('init time', time.time() - start_time)
        bt_msg = bottle_msg()
        while not rospy.is_shutdown():
            try:
                bottle_list = bottle_detector.automated_detect(dtime=2)

                if not len(bottle_list) == 0:
                    bt_msg.num_bottles = len(bottle_list)
                    bt_msg.angles = hd.get_list_info(bottle_list, 'angle')
                    final_bottle = bottle_detector.decision(bottle_list)
                    [bt_msg.final_x_angle, bt_msg.final_y_angle] = final_bottle.get_angle()

                pub.publish(bt_msg)
                rate.sleep()
            except Exception as e:
                print(e)
                pub.publish(bt_msg)
    except rospy.ROSInterruptException:
        pass
