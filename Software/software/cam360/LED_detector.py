"""
This class is responsible to detect the LED's that are used for triangulation.

How should it be organised?
It could be a subclass of HaarDetector
"""
import cv2
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import numpy as np
import time
import math as m
import LED as l
import triangulation as tri

# todo-cf: add robustness: make sure measurements are correct! How?
# with old information, encoders and old angles --> compare
#

class LEDDetector:
    """
    This class starts the color detector and creates LED objects
    
    
    Problems: 
    -  What if color is not exactly detected? Make exposure shift in camera? change hsv region for color?
    -  Maybe include test-run to set the round roi in a nice way!
    -  Put bool variables in config_dict
    """
    # todo-cf: solve problems, see above
    def __init__(self):
        # self.color = color

        # predefined ranges for 4 colors
        # todo-cf: update these color values wit LED-Tests!
        self.color_dict_old = {"blue_low": np.array([100, 120, 200]), "blue_high": np.array([130, 255, 255]),
                           "yellow_low": np.array([10, 20, 150]), "yellow_high": np.array([60, 40, 255]),
                           "green_low": np.array([50, 120, 86]), "green_high": np.array([84, 255, 255]), #was 90-130
                           "red_low": np.array([160, 100, 100]), "red_high": np.array([200, 255, 255])}

        self.color_dict = {"blue_low": np.array([102, 120, 130]), "blue_high": np.array([130, 255, 255]),
                           "yellow_low": np.array([8, 0, 109]), "yellow_high": np.array([29, 30, 255]),
                           "green_low": np.array([65, 120, 126]), "green_high": np.array([84, 255, 255]),  # was 90-130
                           "red_low": np.array([160, 120, 125]), "red_high": np.array([180, 255, 255])}

        self.color_print_dict = {"blue": (176, 0, 0), "yellow": (42, 208, 255),
                                 "green": (42, 173, 255), "red": (51, 62, 255)}

        # dictionary containing all settings
        self.config_dict = {}

        # robot position variables
        # todo-cf: add romans class
        self.position = [0, 0]
        self.angle = []

        # led position variables
        self.pos_reference = []
        #self.pos_blue = None
        #self.pos_green = None
        #self.pos_yellow = None
        #self.pos_red = None

        # LED's
        self.yellow = l.LED("yellow")
        self.blue = l.LED("blue")
        self.green = l.LED("green")
        self.red = l.LED("red")
        self.led_list = [self.yellow, self.blue, self.green, self.red]
        self.angle_list = [self.yellow.angle, self.blue.angle, self.green.angle, self.red.angle]

        # video capture
        self.cap = None
        self.frame = None
        self.gray = None
        self.ret = None
        self.windowsize = [0, 0]
        self.bool_video = False

        # initialize video variables
        self.output_mask = None
        #self.mask_blue = None
        #self.mask_green = None
        #self.mask_red = None
        #self.mask_yellow = None

        #self.blue_contour = None

        # visualisation
        self.bool_visualisation = False
        self.bool_debug_vis = False

        # information of circular ROI
        # todo-cf: maybe implement initialization function with hough circle transform
        self.radius = int(self.windowsize[0]*0.25)
        self.camera_center = [0, 0]

        # init picamera
        self.camera = None
        self.rawCapture = None

        print('Initialization done')

    def video_capture(self):
        """
        How should the video capture be stopped? Time?
        :return: 
        """
        # todo-cf: find stopping mechanism for video_capture: bool_variable?, timer?, etc.

        pass
        """       # initialize rasperry camera
       self.camera = PiCamera()
       self.camera.resolution = (640, 480)
       self.camera.framerate = 32
       self.rawCapture = PiRGBArray(self.camera, size=(640, 480))

       # allow the camera to warmup
       time.sleep(0.1)

       self.bool_video = True
       self.cap = cv2.VideoCapture(0)
       self.windowsize = [self.camera.resolution[0] ,self.camera.resolution[1]]
       self.radius = int(self.windowsize[0] * 0.25)
       self.output_mask = np.zeros((self.windowsize[0], self.windowsize[1]), np.uint8)
       """
        """
        while self.ret and self.bool_video:
            self.ret, self.frame = self.cap.read()
            self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            if not self.ret:
                print('Error: Frame could not be read')
                break
        """

    def init_cam(self):
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        #self.camera.led = False  # turns off the led
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.windowsize = [self.camera.resolution[0], self.camera.resolution[1]]
        self.radius = int(self.windowsize[0] * 0.25)
        self.output_mask = np.zeros((self.windowsize[0], self.windowsize[1]), np.uint8)
        time.sleep(0.1)

    def get_frame(self):
        self.camera.capture(self.rawCapture, format="bgr")
        self.frame = self.rawCapture.array
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.windowsize = self.frame.shape[0:2]

        # truncate manually for right roi
        self.truncate_manually()

        self.rawCapture.truncate(0)

    def set_video(self, state):
        if state is True:
            self.bool_video = True
        elif state is False:
            self.bool_video = False
        else:
            print('Argument state should be True or False')

    def visualize(self):
        """
        Method that visualizes the whole triangulation process. Shows each frame and a mask with four colored lines and
        heading-line on it
        :return: 
        """
        # todo-cf: solve problem with imagesize and displaying the image
        self.bool_visualisation = True
        while self.bool_video and self.bool_visualisation:
            cv2.imshow('360 Camera', self.frame)
            cv2.imshow('Conceptual 360 Camera', self.output_mask)
            #cv2.imshow('Blue Mask', self.blue.mask)
            #cv2.imshow('Circle Test', self.circle_test)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break

        cv2.destroyAllWindows()

    def detect(self):
        self.get_frame()
        self.color_detect()
        self.cvt2position()
        ret = self.validate(self.led_list)

        if ret is not False:
            # todo-cf: add orientation
            self.position = self.triangulation(ret)
        else:
            print('detection not validated: publish old data')

        return self.position

    def det_lines(self):
        # detect hugh lines
        pass

    def color_detect(self):
        # color detection
        while self.bool_debug_vis:
            cv2.imshow('img', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break
        clone_img = self.frame
        hsv = cv2.cvtColor(clone_img, cv2.COLOR_BGR2HSV)

        self.blue.mask = cv2.inRange(hsv, self.color_dict['blue_low'], self.color_dict['blue_high'])
        self.green.mask = cv2.inRange(hsv, self.color_dict['green_low'], self.color_dict['green_high'])
        self.yellow.mask = cv2.inRange(hsv, self.color_dict['yellow_low'], self.color_dict['yellow_high'])
        self.red.mask = cv2.inRange(hsv, self.color_dict['red_low'], self.color_dict['red_high'])

        while False:
            cv2.imshow('original', self.frame)
            cv2.imshow('blue', self.blue.mask)
            cv2.imshow('green', self.green.mask)
            cv2.imshow('yellow', self.yellow.mask)
            cv2.imshow('red', self.red.mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break
        cv2.destroyAllWindows()
        # res = cv2.bitwise_and(clone_img, clone_img, mask=mask)

        # m = cv2.moments(mask, binaryImage=True)
        # todo-cf: morphological transformations, opening and closing!
        # todo-cf: findContours: understand parameters

        # pre processing of color detection:
        kernel1 = np.ones((2, 2), np.uint8)
        kernel2 = np.ones((5, 5), np.uint8)
        kernel3 = np.ones((3, 3), np.uint8)
        for led in self.led_list:

            led.mask = cv2.morphologyEx(led.mask, cv2.MORPH_OPEN, kernel3)
            led.mask = cv2.morphologyEx(led.mask, cv2.MORPH_CLOSE, kernel2)

            #led.mask = cv2.dilate(led.mask, kernel1, iterations=1)
            #led.mask = cv2.morphologyEx(led.mask, cv2.MORPH_OPEN, kernel1)
            ##print(led.mask.shape)
            #led.mask = cv2.dilate(led.mask, kernel2, iterations=1)
            #cv2.imwrite('output_photos/' + str(led.color) + '.jpg', led.mask)

            # contour stuff:
            led.img, led.contours, led.hierarchy = cv2.findContours(led.mask, 1, 2)
            cv2.drawContours(led.mask, led.contours, -1, led.color_tuple, 2)
            print('number of contours for', led, np.size(led.contours))

            if np.size(led.contours) == 0:
                print('No blue LED detected')
                led.position = [None, None]
            else:

                mom_vec = []
                area_vec = []
                # print(contours, hierarchy)
                for i in range(len(led.contours)):
                    mom_vec.append(cv2.moments(led.contours[i], True))
                    area_vec.append(mom_vec[i]['m00'])

                    if area_vec[i] >= 50:
                        cv2.drawContours(clone_img, led.contours, i, (0, 255, 0), 3)
                #print(area_vec)
                #print(mom_vec)

                max_val_area = max(area_vec)
                #print(max_val_area)
                idx = area_vec.index(max_val_area)
                #print(idx)
                led.contour_final = led.contours[idx]
                led.moment_final = mom_vec[idx]

                led.position = [int(led.moment_final['m10'] / led.moment_final['m00']), int(led.moment_final['m01'] / led.moment_final['m00'])]
                cv2.circle(self.frame, tuple(led.position), 4, led.color_tuple, -1)
                cv2.line(self.output_mask, tuple(led.position), tuple(self.camera_center), led.color_tuple,4)
                # print('self.frame shape: ', self.frame.shape)
                # print('self.output_mask shape: ', self.output_mask.shape)
        #cv2.imwrite('output_photos/output_mask.jpg', self.output_mask)

        #
        #
        #
        while True:
            cv2.imshow('original', self.frame)
            cv2.imshow('blue', self.blue.mask)
            cv2.imshow('green', self.green.mask)
            cv2.imshow('yellow', self.yellow.mask)
            cv2.imshow('red', self.red.mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break
        #
        #
        # img_blue, contours_blue, hierarchy_blue = cv2.findContours(self.blue.mask, 1, 2)
        # cv2.drawContours(self.blue.mask, contours_blue, -1, (157, 35, 228), 2)
        #
        #
        # while False:
        #     cv2.imshow('test2', self.blue.mask)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         print('quit successfully')
        #         break
        #
        #
        # print('number of blue contours:', np.size(contours_blue))
        # if np.size(contours_blue) == 0:
        #     print('No blue LED detected')
        #     self.blue.position = [None, None]
        # else:
        #
        #     mom_vec = []
        #     area_vec = []
        #     # print(contours, hierarchy)
        #     for i in range(len(contours_blue)):
        #         mom_vec.append(cv2.moments(contours_blue[i], True))
        #         area_vec.append(mom_vec[i]['m00'])
        #
        #         if area_vec[i] >= 50:
        #             cv2.drawContours(clone_img, contours_blue, i, (0, 255, 0), 3)
        #     print(area_vec)
        #     print(mom_vec)
        #
        #     max_val_area = max(area_vec)
        #     print(max_val_area)
        #     idx = area_vec.index(max_val_area)
        #     print(idx)
        #     self.blue.contour = contours_blue[idx]
        #     m_b = mom_vec[idx]
        #
        #     self.blue.position = [int(m_b['m10'] / m_b['m00']), int(m_b['m01'] / m_b['m00'])]
        #     cv2.circle(self.frame, tuple(self.blue.position), 10, (157, 35, 228), 15)
        #     print('self.frame shape: ', self.frame.shape)
        #     print('self.output_mask shape: ', self.output_mask.shape)
        #
        #     # cv2.drawContours(clone_img, contours_blue, idx, (255, 0, 0), 3)
        #
        # # todo-cf: draw colored line on self.output_mask
        return clone_img

    def cvt2position(self):
        """
        Method that takes the for points and calculates the position of the robot
        
        Problems:
        
        -  Exact shift needed! in order to find the exact angles the pixel of the center need to be known!
        -  
        :return: 
        """
        # todo-cf: what happens if only t

        pos_relative = []
        i = 0
        position_relative = 0
        print('led_list: ', self.led_list)#, [i.position for i in self.led_list])

        for led in self.led_list:
            print("led: ", led)
            if led.position == [None, None]:
                #print("LED is skipped because it hasn't been detected. Led for loop")
                led.pos_relative = [None, None]
                continue

            # shift origin to center and inverse y-direction:
            #print(type(self.camera_center[0]), type(led.position[0]))
            led.position_relative = [led.position[0] - self.camera_center[0], led.position[1] - self.camera_center[1]]
            led.position_relative[1] = - led.position_relative[1]

            print('relative position of led: ', led.position_relative)
            if led.position_relative == None or led.position_relative == [0.0, 0.0]:
                #print("LED has been skipped since it hasn't been detected")
                continue

            # 1st quadrant:
            if led.position_relative[0] >= 0 and led.position_relative[1] >= 0:
                print(led, ' is in 1st quadrant')
                led.angle = (3 * m.pi / 2) + abs(m.atan(led.position_relative[1] / led.position_relative[0]))

            # 2nd quadrant
            elif led.position_relative[0] < 0 and led.position_relative[1] >= 0:
                print(led.color, ' is in 2nd quadrant')
                led.angle = abs(m.atan(led.position_relative[0] / led.position_relative[1]))

            # 3rd quadrant
            elif led.position_relative[0] < 0 and led.position_relative[1] < 0:
                print(led.color, ' is in 3rd quadrant')
                led.angle = (m.pi / 2) + abs(m.atan(led.position_relative[1] / led.position_relative[0]))

            # 4th quadrant
            elif led.position_relative[0] >= 0 and led.position_relative[1] < 0:
                print(led.color, ' is in 4th quadrant')
                #print('angle in 4th', m.degrees(m.atan(led.position_relative[0] / led.position_relative[1])))
                led.angle = (2 * m.pi / 2) + abs(m.atan(led.position_relative[0] / led.position_relative[1]))

            else:
                print('Error in cvt2angle method, LED positions are not right')
                break

            print(led.color, ' angle: ', led.angle)

    def calibrate(self):
        """
        This public method should be used once the picamera is mounted. It sets the variables self.camera_center and 
        self.radius to the right values! 
        :return: 
        """
        self.circle_test = self.gray
        print('self.circle_test shape: ', self.circle_test.shape)
        new_size = tuple([int(i) for i in self.windowsize])
        print('new_size: ', new_size)
        self.circle_test = cv2.resize(self.circle_test, (new_size[1], new_size[0]))
        print('self.circle_test shape: ', self.circle_test.shape)
        #print('self.circle_test successfully created if not None: ', self.circle_test)
        #circles = cv2.HoughCircles(self.gray, cv2.HOUGH_GRADIENT, 1, 100)
        # todo-cf: change minRadius when doing everything with Picamera!
        #circles = cv2.HoughCircles(self.circle_test, cv2.HOUGH_GRADIENT, 10, 1, param1 = 70, param2 = 500,
        #                           minRadius = 100, maxRadius = 0)
        circles = cv2.HoughCircles(self.circle_test, cv2.HOUGH_GRADIENT, 4, 1, param1=70, param2=600,
                                   minRadius=20, maxRadius=1000)

        print('circles detected: ', circles)

        try :
            circles = np.uint16(np.around(circles))
            #print('circle conversion successful: ', circles)
            for c in circles[0,:]:
                # draw the outer circle
                cv2.circle(self.circle_test, (c[0], c[1]), c[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(self.circle_test, (c[0], c[1]), 2, (0, 0, 255), 3)

            while self.bool_debug_vis:
                cv2.imshow('circle test', self.circle_test)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print('quit successfully')
                    break

            npcirc = np.array(circles)
            #print('npcirc: ', npcirc)
            avg = np.mean(npcirc[0][0:1][:], axis=0)
            avg = avg.tolist()
            avg = [int(a) for a in avg]
            print(avg)
            print('before setting circle arguments: ', self.camera_center, self.radius)
            self.camera_center = (avg[0] * 4, avg[1] * 4)
            self.radius = int(avg[2] * 4 * 0.8)
            print('after setting circle arguments: ', self.camera_center, self.radius)
            print('avg: ', avg)
            cv2.circle(self.circle_test, (int(avg[0]), int(avg[1])), int(avg[2]), (255, 255, 255), 3)
        except Exception as e:
            print('Circle detection error: ', e)

    def track(self):
        """not really necessary because color is very clear"""
        pass

    def truncate_manually(self):
        """
        This method should be run to truncate the image and use the right roi!
        :return: 
        """
        # todo-cf: set outer and inner radius before competition!
        # outer_radius = 140
        # inner_radius = 105
        #center = (374, 234) old

        outer_radius = 280
        inner_radius = 170
        center = (562, 478)

        self.camera_center = center

        white = (255, 255, 255)
        black = (0, 0, 0)
        mask_disk = np.zeros((self.windowsize[0], self.windowsize[1], 3), np.uint8)

        cv2.circle(mask_disk, center, outer_radius, white, cv2.FILLED)
        cv2.circle(mask_disk, center, inner_radius, black, cv2.FILLED)

        mask = cv2.inRange(mask_disk, white, white)
        print('truncate active')

        print('mask shape', mask.shape)
        print('frame shape', self.frame.shape)

        # makes everything outside
        self.frame = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        while True:
            cv2.imshow('mask', mask)
            cv2.imshow('img', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break
        cv2.destroyAllWindows()

    def truncate(self):
        """
        Method that truncates the input image and makes it all black around. Idea: Define circle, make mask, use
        
        How should it be activated? 
        :return: 
        """
        # todo-cf: make mask self.mask and set the variables self.radius, self.camera_center once
        # todo-cf: add circle that makes middle part black!

        white = (255, 255, 255)
        mask_disk = np.zeros((self.windowsize[0], self.windowsize[1], 3), np.uint8)

        print('circle arguments: ', self.camera_center, self.radius)

        if self.radius == 0:
            print('Radius is zero!')
        cv2.circle(mask_disk, (int(self.camera_center[0]), int(self.camera_center[1])), self.radius, white, cv2.FILLED)
        mask = cv2.inRange(mask_disk, white, white)
        print('truncate active')

        while self.bool_debug_vis:
            cv2.imshow('mask', mask)
            cv2.imshow('img', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break

        print('mask shape', mask.shape)
        print('frame shape', self.frame.shape)


        # makes everything outside
        self.frame = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        # todo-cf: implement gui where you can place the circle and the radius! maybe add to __init__(self):

    def configure(self):

        # todo-cf: creat configure method that does all the settings
        pass

    def timer(self):
        """
        Timer method where we can measure performance
        :return: 
        """
        # todo-cf: implement timer method
        pass

    def output(self):
        for led in self.led_list:
            if led.position == [None, None]:
                continue
            if led.color == 'test':
                continue
            print(led.color, tuple(self.color_print_dict[led.color]))
            print(type(tuple(self.color_print_dict[led.color])[0]))
            cv2.line(self.output_mask, tuple(self.camera_center), tuple(led.position), led.color_tuple, 10)

    def validate(self, led_list):
        """
        This method checks if the four LED's are in the right order (especially the yellow one!) 
        :return: 
            
        """
        validated = False
        print('validation started')

        # todo-cf: this works only if 4 leds are detected
        loc_led_list = []
        excluded = None
        for led in led_list:
            if led.angle != None:
                loc_led_list.append(led)
            else:
                excluded = led.color

        # sort list: smallest angle first
        sorted_list = sorted(loc_led_list, key=lambda x: x.angle, reverse=False)
        if len(sorted_list) == 4:
            idx = sorted_list.index(self.yellow)
            #print(idx, sorted_list)

            # check if the order of the leds is correct
            next_idx = (idx+1) % len(sorted_list)
            last_idx = (idx-1) % len(sorted_list)
            if sorted_list[next_idx].color == "red":
                if sorted_list[last_idx].color == "blue":
                    validated = True
                    print('angles of led list validated')

            if validated is False:
                diff_list = []
                for ii in range(len(sorted_list)):
                    diff = sorted_list[(ii + 1) % len(sorted_list)].angle - sorted_list[ii].angle

                    if ii == 3: # make sure last angle works as well nicer would be modulus
                        diff = diff + 2 * m.pi
                    diff_list.append(diff)
                min_val = min(diff_list)
                idx = diff_list.index(min_val)

                if diff_list[(idx + 1) % len(sorted_list)] <= diff_list[(idx - 1) % len(sorted_list)]:
                    excluded = sorted_list[(idx + 1) % len(sorted_list)].color
                    sorted_list.pop((idx + 1) % len(sorted_list))
                    print('popped --> continue with 3 leds')
                else:
                    excluded = sorted_list[idx].color
                    sorted_list.pop(idx)
                    print('popped --> continue with 3 leds')

        if len(sorted_list) == 3:
            print('check for 3 leds: ')
            print(sorted_list)
            print('excluded: ', excluded)
            if excluded == "yellow":
                idx = sorted_list.index(self.blue)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "red":
                    print('green is next')
                    if sorted_list[last_idx].color == "green":
                        print('red is last')
                        validated = True
                        print('angles of led list validated')
            elif excluded == "blue":
                idx = sorted_list.index(self.red)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "green":
                    if sorted_list[last_idx].color == "yellow":
                        self.validated = True
                        print('angles of led list validated')
            elif excluded == "red":
                idx = sorted_list.index(self.blue)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "yellow":
                    if sorted_list[last_idx].color == "green":
                        validated = True
                        print('angles of led list validated')
            elif excluded == "green":
                idx = sorted_list.index(self.blue)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "yellow":
                    if sorted_list[last_idx].color == "red":
                        validated = True
                        print('angles of led list validated')
        else:
            validated = False
            print('not enough leds detected')

            # todo-cf: raise error with ros such that old value is published

        if validated is True:
            print('sorted_list returned')

            return sorted_list
        else:
            print('validated ? : ', validated)
            return validated

    def triangulation(self, led_list):

        triang = tri.Triangulation(led_list)
        pos = triang.run()
        return pos

    def test_run(self, path):

        # initialization

        print('Test run started')
        self.frame = cv2.imread(path, cv2.IMREAD_COLOR)
        #self.frame = cv2.resize(self.frame, (648, 484))

        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        print('frame shape: ', self.frame.shape)
        self.windowsize = self.frame.shape[0:2]
        print(self.windowsize[0:2])
        print(self.frame.shape)
        #self.windowsize = (2592, 1936)

        self.radius = int(self.windowsize[0] / 4)
        self.output_mask = np.zeros((self.windowsize[0], self.windowsize[1], 3), np.uint8)

        # initialization done

        while self.bool_debug_vis:
            cv2.imshow('test_run', self.frame)
            cv2.imshow('test_run output_mask', self.output_mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('quit successfully')
                break

        #self.calibrate()
        #self.truncate()
        self.truncate_manually()
        # h_low = 5
        # h_high = 33
        # s_low = 106
        # s_high = 255
        # v_low = 103
        # v_high = 255
        # kernel = np.ones((3, 3), np.uint8)
        #
        # lower_bg = np.array([h_low, s_low, v_low])
        # upper_bg = np.array([h_high, s_high, v_high])
        #
        # hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        # opening = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
        # closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        # bg_mask = cv2.inRange(closing, lower_bg, upper_bg)
        #
        # bg_mask = cv2.bitwise_not(bg_mask)
        #
        # img = cv2.bitwise_and(self.frame, self.frame, mask=bg_mask)
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # opening = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
        # closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        # self.frame = closing

        #select_hsv(self.frame)
        #cv2.imwrite('output_photos/truncated_img.jpg', self.frame)
        #self.set_video(True)
        self.color_detect()

        # # test LED
        # self.test = l.LED('test')
        # self.led_list.append(self.test)
        # self.test.position = [self.camera_center[0] + 100, self.camera_center[1] - 100]

        self.cvt2position()
        # if not self.blue.position == [None, None]:
        #     print('Blue angle: ', m.degrees(self.blue.angle))
        #     print('Blue position: ', self.blue.position)
        #     print('test angle: ', m.degrees(self.test.angle))
        #     print(self.color_print_dict[self.blue.color])
        #self.yellow.angle = None
        ret = self.validate(self.led_list)

        #self.output()
        #self.visualize()

        if ret is not False:
            # todo-cf: add orientation
            print('triangulation started')
            print('ret: ', ret)
            self.position = self.triangulation(ret)
        else:
            print('detection not validated: publish old data')

        print('position: ', self.position)

    def debug_run(self, path):
        self.frame = cv2.imread(path, cv2.IMREAD_COLOR)
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        print('frame shape: ', self.frame.shape)
        self.windowsize = self.frame.shape[0:2]
        print(self.windowsize[0:2])
        print(self.frame.shape)
        # self.windowsize = (2592, 1936)

        self.radius = int(self.windowsize[0] / 4)
        self.output_mask = np.zeros((self.windowsize[0], self.windowsize[1], 3), np.uint8)

def select_hsv(image):
    kernel = np.ones((3, 3), np.uint8)

    img_bg = image

    # preprocessing

    h_low = 5
    h_high = 33
    s_low = 106
    s_high = 255
    v_low = 103
    v_high = 255

    lower_bg = np.array([h_low, s_low, v_low])
    upper_bg = np.array([h_high, s_high, v_high])

    hsv = cv2.cvtColor(img_bg, cv2.COLOR_BGR2HSV)
    opening = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    bg_mask = cv2.inRange(closing, lower_bg, upper_bg)

    bg_mask = cv2.bitwise_not(bg_mask)

    img = cv2.bitwise_and(img_bg, img_bg, mask=bg_mask)

    # TODO FAIRE POUR ENREGISTREMENT VIDEO RASPBERRY
    while (1):

        # _, img = cap.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        opening = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        cv2.createTrackbar('H_low', 'mask', 0, 255, nothing)
        cv2.createTrackbar('H_high', 'mask', 0, 255, nothing)

        cv2.createTrackbar('S_low', 'mask', 0, 255, nothing)
        cv2.createTrackbar('S_high', 'mask', 0, 255, nothing)

        cv2.createTrackbar('V_low', 'mask', 0, 255, nothing)
        cv2.createTrackbar('V_high', 'mask', 0, 255, nothing)

        h_low = cv2.getTrackbarPos('H_low', 'mask')
        h_high = cv2.getTrackbarPos('H_high', 'mask')

        s_low = cv2.getTrackbarPos('S_low', 'mask')
        s_high = cv2.getTrackbarPos('S_high', 'mask')

        v_low = cv2.getTrackbarPos('V_low', 'mask')
        v_high = cv2.getTrackbarPos('V_high', 'mask')

        key = cv2.waitKey(1) & 0xFF

        if key == ord("p"):
            print('h_low =', h_low)
            print('h_high =', h_high)
            print('s_low =', s_low)
            print('s_high =', s_high)
            print('v_low =', v_low)
            print('v_high =', v_high)

        # define range of color in HSV
        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])

        # Threshold the HSV image to get only gray colors
        mask = cv2.inRange(closing, lower, upper)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)

        # for white in mask
        # on a possiblement un obstacle
        # confirmation avec canny detector
        # cv2.imshow('mask',mask)
        # cv2.imshow('frame',frame)
        cv2.imshow('mask', res)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()

def nothing(x):
    pass

def morph(image):
    pass

led_det = LEDDetector()
#led_det.test_run('example_photos/arena/test1/3.jpg')
led_det.test_run('example_photos/arena/test6/3.jpg')
