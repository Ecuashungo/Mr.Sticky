import cv2
import numpy as np
import time
print(cv2.__version__)

width = 600
height = 480
radius = 150

img = cv2.imread('example_photos/3.jpg')

img = cv2.resize(img, (width, height))

mask2 = np.zeros((height,width,3), np.uint8)

cv2.imshow('original', img)
# cap = cv2.VideoCapture(0)
kernel = np.ones((5,5),np.uint8)
#
white = (255, 255, 255)

cv2.circle(mask2, (int(width/2), int(height/2)), radius, white, cv2.FILLED)
mask3 = cv2.inRange(mask2, white, white)
img_final = cv2.bitwise_and(img,img,mask= mask3)

def nothing(x):
    pass

while(1):
#
#     # Take each frame
#     _, frame = cap.read()
#
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #opening = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
    #closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    cv2.createTrackbar('H_low', 'original', 0, 255, nothing)
    h_low = cv2.getTrackbarPos('H_low', 'original')
    cv2.createTrackbar('H_high', 'original', 0, 255, nothing)
    h_high = cv2.getTrackbarPos('H_high', 'original')
    cv2.createTrackbar('S_low', 'original', 0, 255, nothing)
    s_low = cv2.getTrackbarPos('S_low', 'original')
    cv2.createTrackbar('S_high', 'original', 0, 255, nothing)
    s_high = cv2.getTrackbarPos('S_high', 'original')
    cv2.createTrackbar('V_low', 'original', 0, 255, nothing)
    v_low = cv2.getTrackbarPos('V_low', 'original')
    cv2.createTrackbar('V_high', 'original', 0, 255, nothing)
    v_high = cv2.getTrackbarPos('V_high', 'original')

    key = cv2.waitKey(1) & 0xFF

    if key == ord("p"):
        print('The value of H_low is:',h_low)
        print('The value of H_high is:',h_high)
        print('The value of S_low is:',s_low)
        print('The value of S_high is:',s_high)
        print('The value of V_low is:',v_low)
        print('The value of V_high is:',v_high)

    # define range of color in HSV
    lower = np.array([h_low,s_low,v_low])
    upper = np.array([h_high,s_high,v_high])


    # Threshold the HSV image to get only gray colors
    mask = cv2.inRange(hsv, lower, upper)


    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img,img, mask= mask)


    #for white in mask
    # on a possiblement un obstacle
    #confirmation avec canny detector
    cv2.imshow('img', img)
    cv2.imshow('original', res)
    cv2.imshow('mask2', mask2)
    cv2.imshow('mask3', mask3)
    cv2.imshow('img_final', img_final)
    #cv2.imshow('frame',frame)
    #cv2.imshow('res',res)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()