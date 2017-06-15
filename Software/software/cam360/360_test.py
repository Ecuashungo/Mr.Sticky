import cv2
import numpy as np
import time
import LED_detector as ld


# initialize Detector and Picamera
LED_Detector = ld.LEDDetector()
LED_Detector.init_cam()

# determing position here!
pos_360 = LED_Detector.detect()