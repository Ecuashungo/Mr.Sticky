import cv2
import numpy as np
import time

width = 600
height = 480

img = cv2.imread('example_photos/3.jpg')

img = cv2.resize(img, (width, height))
# if img == None:
#    print("Image couldn't be read")
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
blue_low = np.array([100, 120, 200])
blue_high = np.array([130, 255, 255])

mask = cv2.inRange(img_hsv, blue_low, blue_high)
res = cv2.bitwise_and(img, img, mask=mask)

# some preprocessing with closing and opening
kernel1 = np.ones((2,2),np.uint8)
kernel2 = np.ones((5,5),np.uint8)
kernel3 = np.ones((8,8),np.uint8)

opening1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel2)

opening2 = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel1)
erosion = cv2.erode(mask,kernel1,iterations = 1)

closing2 = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, kernel3)

dilation = cv2.dilate(opening1,kernel2,iterations = 1)

cv2.line(img_hsv, (10,10), (100,100), (100, 255, 255), 10)
cv2.line(img_hsv, (100,100), (200,200), (130, 255, 255), 10)



while True:
    cv2.imshow('original', img_hsv)
    cv2.imshow('mask', mask)
    cv2.imshow('closing', closing)
    cv2.imshow('opening1', opening1)
    cv2.imshow('opening2', opening2)
    cv2.imshow('dilation', dilation)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print('quit successfully')
        break
cv2.destroyAllWindows()

