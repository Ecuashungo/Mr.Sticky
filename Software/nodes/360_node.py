#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
sys.path.append("/home/odroid/sticky-robot/software/cam360")
import LED_detector2 as ld
from sticky_robot.msg import pos_360_msg


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('pos_360', pos_360_msg, queue_size=10)
        rospy.init_node('leddetector', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        # initialize Detector and Picamera
        LED_Detector = ld.LEDDetector()
        LED_Detector.init_cam()

        pos_360_msg = pos_360_msg()
        while not rospy.is_shutdown():
            try:
                # determing position here!
                pos_360 = LED_Detector.detect()

                pos_360_msg.header.stamp = rospy.Time.now()
                pos_360_msg.x = pos_360[0]
                pos_360_msg.y = pos_360[1]
                pos_360_msg.theta = pos_360[2]
                pos_360_msg.angle_home = 0
                # todo-CF: implement orientation and angle to yellow LED
                pub.publish(pos_360_msg)
                rate.sleep()
            except Exception as e:
                print(e)
                pub.publish(pos_360_msg)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()