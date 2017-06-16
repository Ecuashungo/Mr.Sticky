import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import math as m
from random import getrandbits
import os

import LED as l
import triangulation as tri


## FUNCTIONS:

def visualize(image_list):
    """
    This function works
    :param image_list: list with images to be visualized
    :return: 
    """
    if not isinstance(image_list, list):
        image_list = [image_list]

    while True:
        i = 0  # for the names of the windows
        for img in image_list:
            cv2.imshow(str(i), img)
            i = i + 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('quit successfully \n')
            break
    cv2.destroyAllWindows()

def morph(image, kernel_size, mode):
    """
    makes opening or closing
    :param img: 
    :param kernel_size: 
    :param mode: either "oc": opening then closing
                        "dod": dilate, open and dilate
    :return: 
    """
    img = image
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    if mode == 'oc':
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        ret = closing

    elif mode == 'dod':
        dilate = cv2.dilate(img, kernel, iterations=1)
        opening = cv2.morphologyEx(dilate, cv2.MORPH_OPEN, kernel)
        dilate2 = cv2.dilate(opening, kernel, iterations=1)
        ret = dilate2

    elif mode == 'dodd':
        dilate = cv2.dilate(img, kernel, iterations=1)
        opening = cv2.morphologyEx(dilate, cv2.MORPH_OPEN, kernel)
        dilate2 = cv2.dilate(opening, np.ones((kernel_size + 1, kernel_size + 1), np.uint8), iterations=1)
        ret = dilate2

    elif mode == 'nothing':
        ret = img

    return ret

def triangulation(led_list):
    triang = tri.Triangulation(led_list)
    pos = triang.run()
    return pos

def auto_canny(image, sigma=0.33):
    """
    Function that calculates a canny edge detection with quite good parameters
    :param image: 
    :param sigma: 
    :return: 
    """
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

class LEDDetector:
    """
    This class starts the color detector and creates LED objects, calculates the angles between the leds and finally 
    calculates the position and orientation of the robot through the triangulation funciton.

    """

    # todo-cf: solve problems, see above
    def __init__(self):
        # self.color = color

        # predefined ranges for 4 colors
        # todo-cf: update these color values wit LED-Tests!
        self.color_dict = {"blue_low": np.array([100, 120, 200]), "blue_high": np.array([130, 255, 255]),
                           "yellow_low": np.array([10, 0, 150]), "yellow_high": np.array([60, 40, 255]),
                           "green_low": np.array([65, 120, 86]), "green_high": np.array([90, 255, 255]),  # was 90-130
                           "red_low": np.array([160, 100, 100]), "red_high": np.array([200, 255, 255])}

        self.color_print_dict = {"blue": (176, 0, 0), "yellow": (42, 208, 255),
                                 "green": (42, 173, 255), "red": (51, 62, 255)}

        # dictionary containing all settings
        self.config_dict = {}

        # robot position variables
        # todo-cf: add romans class
        self.position = [None, None, None]  # [x, y, theta]

        # led position variables
        self.pos_reference = []

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
        self.hsv = None
        self.ret = None
        self.windowsize = [0, 0]
        self.bool_video = False

        # initialize video variables
        self.output_mask = None

        # visualisation
        self.bool_visualisation = False
        self.bool_debug_vis = False

        # information of circular ROI
        # todo-cf: maybe implement initialization function with hough circle transform
        self.radius = int(self.windowsize[0] * 0.25)
        self.camera_center = [0, 0]

        # init picamera
        self.camera = None
        self.rawCapture = None
        
        self.counter = 0
        print('Initialization done')

    def init_cam(self, visualisation=False, verbose=False):
        # todo-cf: test individually
        self.camera = PiCamera()
        self.camera.resolution = (1080, 960)

        self.rawCapture = PiRGBArray(self.camera)
        self.windowsize = [self.camera.resolution[1], self.camera.resolution[0]]
        if verbose:
            print("self.windowsize: ", self.windowsize)
        self.radius = int(self.windowsize[0] * 0.25)
        self.output_mask = np.zeros((self.windowsize[0], self.windowsize[1]), np.uint8)
        time.sleep(0.1)
        self.camera.capture(self.rawCapture, format="bgr")
        self.frame = self.rawCapture.array
        if verbose:
            print("self.windowsize: ", self.windowsize)
            print("shape frame after rawCapture.array", self.frame.shape)
        self.rawCapture.truncate(0)
        self.truncate(visualisation=visualisation, verbose=verbose)
        if visualisation:
            print("self.frame before flip")
            visualize(self.frame)
        self.flip()
        if visualisation:
            print("self.frame after flip")
            visualize(self.frame)

    def get_frame(self, verbose=False, visualisation=False):
        # todo-cf: test individually
        self.camera.capture(self.rawCapture, format="bgr")
        self.frame = self.rawCapture.array
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.windowsize = list(self.frame.shape[0:2])
        if visualisation:
            print('self.frame after reading from picam: ')
            visualize([self.frame])
        if verbose:
            if self.frame is None:
                print('self.frame is None')

        # truncate manually for right roi
        self.truncate(verbose=verbose, visualisation=visualisation)
        self.flip()

        self.rawCapture.truncate(0)

    def color_mask(self, visualisation=False):
        """
        This method creates the colormasks of the leds in self.led_list 
        :return: 
        """
        img = self.frame
        self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # create color masks
        self.blue.mask = cv2.inRange(self.hsv, self.color_dict['blue_low'], self.color_dict['blue_high'])
        self.green.mask = cv2.inRange(self.hsv, self.color_dict['green_low'], self.color_dict['green_high'])
        self.yellow.mask = cv2.inRange(self.hsv, self.color_dict['yellow_low'], self.color_dict['yellow_high'])
        self.red.mask = cv2.inRange(self.hsv, self.color_dict['red_low'], self.color_dict['red_high'])

        if visualisation:
            print('self.frame: ')
            visualize([self.frame])
            print('blue mask: ')
            visualize([self.blue.mask])
            print('green mask: ')
            visualize([self.green.mask])
            print('yellow mask: ')
            visualize([self.yellow.mask])
            print('red mask: ')
            visualize([self.red.mask])

    def process(self, kernel_size=3, mode='oc'):
        """
        This method processes the color_masks in self.led_list
        :param kernel_size: size of the kernel for the morphological transformation
        :param mode: "oc": opening/closing."dod": dilate/opening/dilate. "nothing" if no processing is desired
        :return: 
        """
        try:
            for led in self.led_list:
                led.mask_processed = morph(led.mask, kernel_size, mode)
        except Exception as e:
            for led in self.led_list:
                led.mask_processed = led.mask
            print('Error in process method. Error message: ', e)

    def moment_search(self, verbose=False, visualisation=False, presentation=False, save=False):
        """
        method that takes a mask and looks for moments 
        :return: 
        """
        
        if save:
            print('in save mode')
            cv2.imwrite(os.path.join(os.path.expanduser('~'),'sticky-robot','software', 'cam360', 'frames', str(self.counter) + '.bmp'), self.frame)
            self.counter = self.counter + 1
        if verbose:
            print('\n')
            print('\n moment_search method: \n')
        try:
            for led in self.led_list:

                led.img_color = self.frame

                led.img, led.contours, led.hierarchy = cv2.findContours(led.mask_processed, 1, 2)
                num_contours = len(led.contours)
                print(num_contours)
                if visualisation:
                    cv2.drawContours(led.img_color, led.contours, -1, led.color_tuple, 2)
                if verbose:
                    print('Number of contours for', led, num_contours)

                if num_contours == 0:
                    led.position = [None, None]
                    if verbose:
                        print('Position of ', led, ' set to: [None, None]')
                else:
                    mom_vec = []
                    area_vec = []
                    for i in range(num_contours):  # calculate moments and areas of the contours
                        mom_vec.append(cv2.moments(led.contours[i], True))
                        area_vec.append(mom_vec[i]['m00'])

                    # find contour with largest area
                    max_area = max(area_vec)
                    idx = area_vec.index(max_area)

                    if max_area >= 3.0:
                        # update the led's position
                        led.contour_final = led.contours[idx]
                        led.moment_final = mom_vec[idx]

                        led.position = [int(led.moment_final['m10'] / led.moment_final['m00']),
                                        int(led.moment_final['m01'] / led.moment_final['m00'])]

                        if verbose:
                            print('Position of ', led, ' set to: ', led.position, '\n')

                        if presentation:
                            cv2.line(self.frame, tuple(led.position), tuple(self.camera_center), led.color_tuple, 4)
                        if save:
                            cv2.line(self.frame, tuple(led.position), tuple(self.camera_center), led.color_tuple, 4)

                    else:
                        print('The detected contours are too small')
                        led.position = [None, None]
                        if verbose:
                            print('Position of ', led, ' set to: [None, None] \n')

            if save:
                print('in save mode 2')
                print('counter: ', self.counter)
                cv2.imwrite(os.path.join(os.path.expanduser('~'),'sticky-robot','software', 'cam360', 'frames_modified', str(self.counter) + '.bmp'), self.frame)
            if presentation:
                visualize([self.frame])
            if visualisation:
                print('self.frame: ')
                visualize([self.frame])
                print('blue mask: ')
                visualize([self.blue.mask])
                print('green mask: ')
                visualize([self.green.mask])
                print('yellow mask: ')
                visualize([self.yellow.mask])
                print('red mask: ')
                visualize([self.red.mask])

        except Exception as e:
            print('Error in moment_search method. Error message: ', e)

    def line_search(self, verbose=False, visualisation=False):
        """
        method that takes a mask and looks for hough_lines
        :return: 
        """
        # FIXME this has not been tested yet
        if verbose:
            print('\n line_search method \n')
        for led in self.led_list:
            led.edge = auto_canny(led.mask_processed, sigma=0.1)
            if visualisation:
                print(led.color, 'canny edge: ')
                visualize([led.edge])

            # FIXME: finish implementation of hough-line-detection
            led.lines = cv2.HoughLines(led.edge, 5, np.pi / 180, 200)
            print(led.lines)
            for rho, theta in led.lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                cv2.line(self.output_mask, (x1, y1), (x2, y2), led.color_tuple, 4)
                print('self.output_mask')
                visualize([self.output_mask])

        pass

    def cirle_intersect(self, line, center, radius):
        """
        method that takes lines and a circle and looks if they intersect
        :param line: 
        :param center: 
        :param radius: 
        :return: 
        """
        pass

    def cvt2position(self, verbose=False):
        """
        This method converts the positions of the LED's into angles. The zero angle is the positive y-axis and 
        the positive direction is counter-clockwise.
        :param verbose: True if verbose mode needed
        :return: 
        """

        if verbose:
            print('\n')
            print('\n cvt2position method: \n')
            print('led_list: ', self.led_list, '\n')  # , [i.position for i in self.led_list])

        for led in self.led_list:

            if led.position == [None, None]:
                if verbose:
                    print(led, " is skipped because led.position: ", led.position, '\n')
                led.pos_relative = [None, None]
                continue

            # shift origin to center and inverse y-direction:
            led.position_relative = [led.position[0] - self.camera_center[0], led.position[1] - self.camera_center[1]]
            led.position_relative[1] = - led.position_relative[1]  # positive y: towards top

            if verbose:
                print('position (in pixels) of ', led, 'is: ', led.position)
                print('relative position of ', led, 'is: ', led.position_relative)

            if led.position_relative == None or led.position_relative == [0.0, 0.0]:
                if verbose:
                    print(led, " has been skipped since it hasn't been detected")
                continue

            # quadrant: for verbose mode: printing
            quadrant = None
            # 1st quadrant:
            if led.position_relative[0] >= 0 and led.position_relative[1] >= 0:
                led.angle = (3 * m.pi / 2) + abs(m.atan(led.position_relative[1] / led.position_relative[0]))
                quadrant = '1st'

            # 2nd quadrant
            elif led.position_relative[0] < 0 and led.position_relative[1] >= 0:
                led.angle = abs(m.atan(led.position_relative[0] / led.position_relative[1]))
                quadrant = '2nd'

            # 3rd quadrant
            elif led.position_relative[0] < 0 and led.position_relative[1] < 0:
                led.angle = (m.pi / 2) + abs(m.atan(led.position_relative[1] / led.position_relative[0]))
                quadrant = '3rd'

            # 4th quadrant
            elif led.position_relative[0] >= 0 and led.position_relative[1] < 0:
                led.angle = (2 * m.pi / 2) + abs(m.atan(led.position_relative[0] / led.position_relative[1]))
                quadrant = '4th'

            else:
                print('Error in cvt2angle method, LED positions are not right')
                break

            if verbose:
                print(led, ' is in ', quadrant, ' quadrant, with angle: ', led.angle, '\n')

    def truncate(self, verbose=False, visualisation=False):
        """
        This method should be run to truncate the image and use the right roi!
        :return: 
        """
        if verbose:
            print('\n truncate method: \n')

        outer_radius = 280
        inner_radius = 195
        center = [557, 486]

        self.camera_center = center

        white = (255, 255, 255)
        black = (0, 0, 0)
        mask_disk = np.zeros((self.windowsize[0], self.windowsize[1], 3), np.uint8)

        cv2.circle(mask_disk, tuple(center), outer_radius, white, cv2.FILLED)
        cv2.circle(mask_disk, tuple(center), inner_radius, black, cv2.FILLED)

        mask = cv2.inRange(mask_disk, white, white)
        if visualisation:
            visualize([mask])

        if verbose:
            print('size self.frame: ', self.frame.shape)
            print('size mask: ', mask.shape)
        # makes everything outside
        self.frame = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        if visualisation:
            print('visualize mask: ')
            visualize([mask])
            print('visualize self.frame: ')
            visualize([self.frame])

    def flip(self, visualisation=False):
        """
        This method flips the frame, such that the image corresponds to the arena as we see it
        :param visualisation: 
        :return: 
        """
        # flip image
        img_flipped = cv2.flip(self.frame, 1)
        self.frame = img_flipped

        # calculate new center
        self.camera_center[0] = self.windowsize[1] - self.camera_center[0]

        if visualisation:
            cv2.line(self.frame, (self.camera_center[0], self.camera_center[1] + 200),
                     (self.camera_center[0], self.camera_center[1] - 200), (255, 255, 255))
            cv2.line(self.frame, (self.camera_center[0] + 200, self.camera_center[1]),
                     (self.camera_center[0] - 200, self.camera_center[1]), (255, 255, 255))
            visualize([self.frame])

    def validate_led_position(self, led_list, verbose=False, visualisation=False):
        """
        This method checks if the four LED's are in the right order (especially the yellow one!) 
        :return: 

        """
        validated = False
        if verbose:
            print('\n validation method: \n')

        loc_led_list = []
        excluded = None
        for led in led_list:
            if led.angle != None:
                loc_led_list.append(led)
            else:
                excluded = led.color

        # sort list: smallest angle first
        sorted_list = sorted(loc_led_list, key=lambda x: x.angle, reverse=False)

        # If there is four LED's --> average triangulation over the combinations
        if len(sorted_list) == 4:
            idx = sorted_list.index(self.yellow)

            # check if the order of the leds is correct
            next_idx = (idx + 1) % len(sorted_list)
            last_idx = (idx - 1) % len(sorted_list)
            if sorted_list[next_idx].color == "blue":
                if sorted_list[last_idx].color == "red":
                    validated = True
                    print('angles of led list validated')

        # if there is only three LED's detected: calculate position
        if len(sorted_list) == 3:
            print('check for 3 leds: ')
            print(sorted_list)
            print('excluded: ', excluded)
            if excluded == "yellow":
                idx = sorted_list.index(self.blue)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "green":
                    print('green is next')
                    if sorted_list[last_idx].color == "red":
                        print('red is last')
                        validated = True
                        print('angles of led list validated')
            elif excluded == "blue":
                idx = sorted_list.index(self.red)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "yellow":
                    if sorted_list[last_idx].color == "green":
                        self.validated = True
                        print('angles of led list validated')
            elif excluded == "red":
                idx = sorted_list.index(self.blue)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "green":
                    if sorted_list[last_idx].color == "yellow":
                        validated = True
                        print('angles of led list validated')
            elif excluded == "green":
                idx = sorted_list.index(self.blue)
                print(idx, sorted_list)
                next_idx = (idx + 1) % len(sorted_list)
                last_idx = (idx - 1) % len(sorted_list)
                if sorted_list[next_idx].color == "red":
                    if sorted_list[last_idx].color == "yellow":
                        validated = True
                        print('angles of led list validated')
        elif len(sorted_list) <= 2:
            validated = False
            print("not enough LED's detected")

        if validated is True:
            print('sorted_list returned')

            return sorted_list
        elif validated is not True:
            print('Detection is not validated: ')
            return validated

    def validate_final_position(self):
        validate = False
        if self.position[0] <= 8.0 and self.position[0] >= 0.0:
            validate = True
        if self.position[1] <= 8.0 and self.position[1] >= 0.0:
            validate = True
        return validate

    def detect(self, verbose=False, visualisation=False, presentation=False, save=False):
        
        if verbose:
            print('\n  detect Method:  \n')

        self.get_frame(verbose=verbose, visualisation=visualisation)
        self.color_mask(visualisation=visualisation)
        self.process(kernel_size=3, mode='oc')
        self.moment_search(verbose=verbose, visualisation=visualisation, presentation=presentation, save=save)
        self.cvt2position(verbose=verbose)
        ret = self.validate_led_position(self.led_list)
        if ret is not False:
            if verbose:
                print('triangulation started')
                print('ret: ', ret)
                print('angles: ', [i.angle for i in ret])
            self.position = triangulation(ret)
        else:
            if verbose:
                print('detection not validated: publish old data')

        print('Final robot position is: ', self.position)

        if save and self.position != [None, None, None]:
            # arena image:
            print('in detection save mode')
            l = 20
            color = (30, 30, 255)
            arena = cv2.imread(os.path.join(os.path.expanduser('~'), 'sticky-robot', 'software', 'cam360', 'arena.png'), cv2.IMREAD_COLOR)
            print('arena: ', arena)
            arena = cv2.resize(arena, (400, 400)) 
            cv2.arrowedLine(arena,(int(50*self.position[0]), int(400-50*self.position[1])), (int(50*self.position[0] + l * m.sin(self.position[2])),int(400 - 50 * self.position[1] + l * m.cos(self.position[2]))), color)
            cv2.imwrite(os.path.join(os.path.expanduser('~'), 'sticky-robot', 'software', 'cam360', 'map', str(self.counter) + '.bmp'), arena)
        if False:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.frame, 'x: ' + str(self.position[0]) + 'y: ' + str(self.position[1]), font, 1,
                        (200, 255, 155), 2, cv2.LINE_AA)
            print('Robot Position: \n')
            print('X:', self.position[0])
            print('Y:', self.position[1])
            print('Theta:', self.position[2])

        return self.position

    def output_image(self):
        """
        if called it creates the output mask and saves it
        :return: 
        """
        for led in self.led_list:
            cv2.line(self.output_mask, tuple(led.position), tuple(self.camera_center), led.color_tuple, 4)

    def presentation(self):
        print('Presentation method')
        print('after arena')
        # 360 cam
        for led in self.led_list:
            cv2.line(self.frame, tuple(led.position), tuple(self.camera_center), led.color_tuple, 2)
        
        output_image = cv2.resize(self.frame, (int(self.windowsize[1] / 2.5), int(self.windowsize[0] / 2.5)))

        cv2.imshow('Triangulation', output_image)
        print('end of presentation method')

def test():
    led_det = LEDDetector()
    img1 = cv2.imread('example_photos/calibration/17.bmp')
    img2 = cv2.imread('example_photos/calibration/18.bmp')
    img3 = cv2.imread('example_photos/calibration/19.bmp')
    img4 = cv2.imread('example_photos/calibration/17.bmp')
    img5 = cv2.imread('example_photos/arena_better_leds/test1/3.jpg')
    img6 = cv2.imread('example_photos/arena_better_leds/test2/3.jpg')
    img7 = cv2.imread('example_photos/arena_better_leds/test3/3.jpg')
    img8 = cv2.imread('example_photos/arena_better_leds/test4/3.jpg')
    img9 = cv2.imread('example_photos/arena_better_leds/test5/3.jpg')


    led_det.frame = img2
    led_det.windowsize = list(led_det.frame.shape[0:2])
    #led_det.detect(verbose=True, visualisation=True, presentation=True, save=False)
    #led_det.truncate(verbose=True, visualisation=False)
    #led_det.flip(visualisation=True)

    led_det.camera_center = [557, 486]
    led_det.camera_center[0] = led_det.windowsize[1] - led_det.camera_center[0]
    # led_det.camera_center = [center[0]-led_det.camera_center[0]+center[0], center[1]-led_det.camera_center[1]]+center[1])
    print(led_det.camera_center)


    led_det.color_mask(visualisation=False)
    led_det.process(kernel_size=3, mode='oc')


    #led_det.line_search(verbose=True, visualisation=True)
    led_det.moment_search(verbose=True, visualisation=False, presentation=True)
    led_det.cvt2position(verbose=True)
    ret = led_det.validate_led_position(led_det.led_list)
    if ret is not False:
        # todo-cf: add orientation
        print('triangulation started')
        print('ret: ', ret)
        print('angles: ', [i.angle for i in ret])
        led_det.position = triangulation(ret)
    else:
        print('detection not validated: publish old data')


    print('Final robot position is: ', led_det.position)


