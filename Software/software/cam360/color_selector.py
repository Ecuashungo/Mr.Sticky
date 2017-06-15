import cv2
import numpy as np

image = cv2.imread('example_photos/arena/test1/3.jpg')
print(image)
#r = 500.0 / image.shape[1]
#dim = (500, int(image.shape[0] * r))

# perform the actual resizing of the image and show it
img = cv2.resize(image, (600, 480))
cv2.imshow('original', img)
# cap = cv2.VideoCapture(0)
kernel = np.ones((5,5),np.uint8)
#
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

    cv2.createTrackbar('H_low', 'mask', 0, 255, nothing)
    h_low = cv2.getTrackbarPos('H_low', 'mask')
    cv2.createTrackbar('H_high', 'mask', 0, 255, nothing)
    h_high = cv2.getTrackbarPos('H_high', 'mask')
    cv2.createTrackbar('S_low', 'mask', 0, 255, nothing)
    s_low = cv2.getTrackbarPos('S_low', 'mask')
    cv2.createTrackbar('S_high', 'mask', 0, 255, nothing)
    s_high = cv2.getTrackbarPos('S_high', 'mask')
    cv2.createTrackbar('V_low', 'mask', 0, 255, nothing)
    v_low = cv2.getTrackbarPos('V_low', 'mask')
    cv2.createTrackbar('V_high', 'mask', 0, 255, nothing)
    v_high = cv2.getTrackbarPos('V_high', 'mask')

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
    cv2.imshow('mask',mask)
    #cv2.imshow('frame',frame)
    #cv2.imshow('res',res)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()