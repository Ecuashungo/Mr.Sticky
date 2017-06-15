import cv2
import numpy as np

path = 'example_photos/arena/test6/3.jpg'
camera_center = [562, 478]
radius = 30


print('Test run started')
frame = cv2.imread(path, cv2.IMREAD_COLOR)
#self.frame = cv2.resize(self.frame, (648, 484))

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
print('frame shape: ', frame.shape)
windowsize = frame.shape[0:2]
print(windowsize[0:2])
print(frame.shape)
#self.windowsize = (2592, 1936)

#radius = int(windowsize[0] / 4)
output_mask = np.zeros((windowsize[0], windowsize[1], 3), np.uint8)

circle_test = gray
print('self.circle_test shape: ', circle_test.shape)
new_size = tuple([int(i) for i in windowsize])
print('new_size: ', new_size)
circle_test = cv2.resize(circle_test, (new_size[1], new_size[0]))
print('self.circle_test shape: ', circle_test.shape)
# print('self.circle_test successfully created if not None: ', self.circle_test)
# circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT, 1, 100)
# todo-cf: change minRadius when doing everything with Picamera!
# circles = cv2.HoughCircles(self.circle_test, cv2.HOUGH_GRADIENT, 10, 1, param1 = 70, param2 = 500,
#                           minRadius = 100, maxRadius = 0)
circles = cv2.HoughCircles(circle_test, cv2.HOUGH_GRADIENT, 4, 1, param1=70, param2=800,
                           minRadius=20, maxRadius=1000)

print('circles detected: ', circles)

try:

    # circles = np.uint16(np.around(circles))
    # # print('circle conversion successful: ', circles)
    # for c in circles[0, :]:
    #     # draw the outer circle
    #     cv2.circle(circle_test, (c[0], c[1]), c[2], (0, 255, 0), 2)
    #     # draw the center of the circle
    #     cv2.circle(circle_test, (c[0], c[1]), 2, (0, 0, 255), 3)
    #
    # while True:
    #     cv2.imshow('circle test', circle_test)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         print('quit successfully')
    #         break

    cv2.line(frame, (564, 10), (564, 1000), (255,255,255), 2)
    cv2.line(frame, (10, 482), (900, 482), (255,255,255), 2)

    while True:
        cv2.imshow('verifyq', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('quit successfully')
            break

    npcirc = np.array(circles)
    # print('npcirc: ', npcirc)
    avg = np.mean(npcirc[0][0:1][:], axis=0)
    avg = avg.tolist()
    avg = [int(a) for a in avg]
    print(avg)
    print('before setting circle arguments: ', camera_center, radius)
    camera_center = (avg[0] * 4, avg[1] * 4)
    radius = int(avg[2] * 4 * 0.8)
    print('after setting circle arguments: ', camera_center, radius)
    print('avg: ', avg)
    cv2.circle(circle_test, (int(avg[0]), int(avg[1])), int(avg[2]), (255, 255, 255), 3)
except Exception as e:
    print('Circle detection error: ', e)