import cv2
import numpy as np
#import LED_detector as ld


img = cv2.imread('output_photos/red.jpg', cv2.IMREAD_GRAYSCALE)
img_copy = cv2.imread('output_photos/red.jpg', cv2.IMREAD_GRAYSCALE)
#gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#edges = cv2.Canny(gray,50,150,apertureSize = 3)

lines = cv2.HoughLines(img, 40, np.pi/180, 2)
print('lines: ', lines)
print('lines: ', lines)
try:
    for [[rho, theta]] in lines:
        print(rho, theta)
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),6)
        print('line printed')

    while True:
        cv2.imshow('houghlines', img)
        cv2.imshow('original', img_copy)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('quit successfully')
            break

except Exception as e:
    print('Error> ', e)


