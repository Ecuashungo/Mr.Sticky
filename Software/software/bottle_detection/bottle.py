import numpy as np
from random import randint
from collections import deque

# todo-cf: add bool_variable for printing (-verbose mode), and remove unnecessary prints
# todo-cf: maybe store w,h in detections as well and calculate average, to use for decision

class Bottle:
    def __init__(self, initial, imagesize, start_time, name='bottle'):
        self.bottle = np.array([initial])  # numpy array with detections of the bottle, initial is the first detection
        self.status = True  # can the instance be considered as a bottle or is it an outlier?
        self.mean = np.rint(np.mean(self.bottle, axis=0))  # mean of bottle detection
        self.angle = np.array([])  # angular position of the mean, in degrees
        self.eps = 0.05  # epsilon such that detection is counted as the same bottle
        self.imagesize = np.rint(imagesize)  # image dimensions that are being analyzed: [width, height] or np.array
        self.name = name
        self.start_time = start_time
        self.last_detection_time = start_time
        self.detection_list = []
        self.actual_position = []

        # visualization variables:
        self.tracking_color = (randint(0,255), randint(0,255), randint(0,255))
        self.BUFFER_SIZE = 12
        self.vis_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.vis_buffer.appendleft([int(initial[0]), int(initial[1])])
        # initialize the list of tracked points, the frame counter,
        # and the coordinate deltas
        self.tracked = False  # boolean variable that decides if this bottle is tracked or not
        self.Time_THRESHOLD = 10 #FIXME: Need to set this properly!
        print('Name of new bottle is: ', self.name)

    def __repr__(self):
        return "%s" % (self.name)

    def __str__(self):
        return "%s" % (self.name)

    def get_status(self):
        return self.status

    def get_detections(self, time_steps):
        detections = self.bottle[-time_steps:]
        mean = np.rint(np.mean(detections, axis=0))
        return mean

    def get_all_detections(self):
        return self.bottle

    def get_mean(self):
        self.mean = np.rint(np.mean(self.bottle, axis=0))  # update mean
        return self.mean

    def get_num_det(self):
        return np.size(self.bottle, axis=0)

    def get_windowsize(self):
        return self.imagesize

    def get_dist(self, detection):
        """
        This method calculates 
        :param detection: 
        :return: 
        """
        dist = np.sqrt(np.sum((self.bottle - detection) ** 2, axis=1))
        idx_min = np.argmin(dist)
        min_dist = dist[idx_min]
        return min_dist

    def get_info(self):
        """
        This method returns information about the object in form of a list
        :return: [name, list of detections, number of detections, mean, windowsize]
        """
        return [self.name, self.bottle, np.size(self.bottle, axis=0), self.mean, self.imagesize]

    def get_name(self):
        return self.name

    def get_angle(self):
        return self.angle

    def set_eps(self, eps):
        self.eps = eps
        print(' '.join(['Epsilon set to', str(eps)]))

    def set_status(self, status):
        if status == 'on' or status == True:
            print('Status of ', self.name, 'set to True')
            self.status = True
        elif status == 'off' or status == False:
            print('Status of ', self.name, 'set to False')
            self.status = False

    def part_of_bottle(self, detection):
        if self.get_dist(detection) <= self.eps * self.imagesize[0]:
            print(' '.join(['This detection is part of', self.name]))
            return True
        else:
            print(' '.join(['This detection is not part of', self.name]))
            return False

    def add_detection_manually(self, detection, actual_time_counter):
        self.last_detection_time = actual_time_counter
        print(detection, ' detection is added to ', self.name)
        detection_2d = np.array([detection])  # bring detection into right dimensions
        self.bottle = np.concatenate((self.bottle, detection_2d), axis=0)
        self.actual_position = list(detection)
        self.vis_buffer.appendleft([int(detection[0]), int(detection[1])])


        #print(' '.join(['Detection', detection, 'added to', self.name])) doesnt work! because of detection

    def add_detection(self, detection):  # actually not needed! check if part of bottle outside class
        if self.part_of_bottle(detection):
            self.add_detection_manually(detection)
            return True
        else:
            print('The detection is not part of the bottle')
            return False

    def set_angle(self, angle):
        # todo-cf: test set_angle method
        self.angle = angle
        print('Angle of %s set to: %f' % (self.name, self.angle[0]))
        return True

    def check_age(self, actual_time_counter):
        if actual_time_counter - self.last_detection_time <= self.Time_THRESHOLD:
            return True
        else:
            return False

# todo-cf: transfer imagesize and windowsize to HaarDetector