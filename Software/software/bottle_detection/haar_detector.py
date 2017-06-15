import numpy as np
import cv2
import time
from importlib import import_module


class HaarDetector:
    """
    This class starts a Haar detector, it takes the path to the classifier and the classname of the objects that want to
    be detected.
    """
    # todo-cf: add all doc-strings and nicely describe things!
    # todo-cf: add bool_variable for printing (-verbose mode), and remove unnecessary prints

    def __init__(self, path, classname):
        self.classifier = cv2.CascadeClassifier(str(path))
        self.classname = str(classname)

        # import the right class that corresponds to the classifier
        self.object_class = import_module(self.classname)
        self.class_dictionnary = {'bottle': self.object_class.Bottle}

        self.object_list = []

        # Field of View of camera, standard camera is PiCamera
        #self.fov_deg = [62.2, 48.8]  # horizontal field of view pi camera in degrees
        self.fov_deg = [68, 56]
        self.fov_rad = np.deg2rad(self.fov_deg)  # conversion to radians
        self.windowsize = np.array([])

        # Bool variables with their initial value
        self.bool_visualisation = False
        self.bool_timer = False
        self.bool_objects = True

    def video_capture(self):
        """
        This method starts the video capture.
        :rtype: list
        :return: list with the capture and information about it
        """
        cap = cv2.VideoCapture(0)
        self.windowsize = np.array([cap.get(3), cap.get(4)])
        return cap

    def new_object(self, inputlist):
        # todo-cf: new_object method: add obstacle possibility
        if self.classname == 'bottle':
            return_object = self.object_class.Bottle(inputlist[0], inputlist[1], inputlist[2])
        elif self.classname == 'obstacle':
            """
            define object class and it's neede inputs. Put here so that objects can be detected!
            """
            # return_object =
            pass
        else:
            print("Error: new_object couldn't be created, because ", self.classname, " is not correct")
            return_object = None
        return return_object

    def detect(self, cap, dtime):
        # initialize time
        start_time = time.time()

        # initialize list of objects, where objects are saved
        object_list = []

        # initialize object_counter to name detected objects
        object_counter = 0

        frame_counter = 0

        while time.time() - start_time < dtime:
            if self.bool_timer == True:
                test_time = time.time()

            # Read the caption frame per frame
            ret, img = cap.read()
            frame_counter = frame_counter + 1
            if not ret:
                print("cap.read() didn't work properly")
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert each frame to gray scale

            # detection_list is a vector containing (x,y,w,h) of the detections
            # what are the argumetns 1.3 and 5?
            detection_list = self.classifier.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in detection_list:

                center = np.array([x + w / 2, y + h / 2])  # Calculate center of detection

                # Create Objects for detections - bottles, obstacles, etc.
                if self.bool_objects == True:

                    # If object_list is empty, initialize with first detection
                    if len(object_list) == 0:
                        print('Bottle list is empty')
                        object_list.append(
                            self.new_object([center, self.windowsize, '_'.join(['Bottle', str(object_counter)])]))
                        # object_list.append(bt.Bottle(center, windowsize, '_'.join(['Bottle', str(object_counter)])))
                        object_counter = object_counter + 1
                        print(object_list)
                        print('First bottle added\n')

                    # If object_list not empty, see if new detection belongs to a existing object.
                    else:
                        distance_list = []  # store distances to existing objects to find shortest

                        # Find closest bottle by analysing the shortest distance
                        for b in object_list:
                            distance_list.append(b.get_dist(center))
                        idx = np.argmin(distance_list)  # find index of smallest distance
                        closest_bottle = object_list[idx]  # identify closest bottle

                        # If it belongs to the closest object, append detection to object
                        if closest_bottle.part_of_bottle(center):
                            closest_bottle.add_detection_manually(center)
                            object_list[idx] = closest_bottle
                            if self.bool_visualisation == True:
                                cv2.putText(img, closest_bottle.get_name(), (x, int(y - 0.03 * self.windowsize[1])),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255))

                        # If it does not belong to an object, create a new one
                        elif not closest_bottle.part_of_bottle(center):
                            object_list.append(
                                self.new_object([center, self.windowsize, '_'.join(['Bottle', str(object_counter)])]))
                            # object_list.append(bt.Bottle(center, self.windowsize, '_'.join(['Bottle', str(object_counter)])))
                            object_counter = object_counter + 1
                            if self.bool_visualisation == True:
                                cv2.putText(img, object_list[-1].get_name(), (x, int(y - 0.03 * self.windowsize[1])),
                                            cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 255))
                            print('Bottle added')

                # drawing on image:
                if self.bool_visualisation == True:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3)

            # Print run time if needed
            if self.bool_timer == True:
                print('time for 1 iteration: ', test_time - time.time())

            if self.bool_visualisation == True:
                cv2.imshow('Bottle Detection', img)

                # ability to quit the visualisation and the detection
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print('quit successfully')
                    break

        cap.release()
        print('frames analyzed: ', frame_counter)

        if self.bool_visualisation == True:
            cv2.destroyAllWindows()

        return object_list

    def automated_detect(self, dtime=10):
        """
        Starts capture and detection function and validates them. Calculates the angles of the bottles and prints the
        whole information nicely
        :return: detection_list
        """
        # starts caption
        cap = self.video_capture()
        # runs detection:
        object_list = self.detect(cap, dtime)
        # validates detected bottles
        object_list_validated = self.validate(object_list)
        # Print results nicely
        self.object_list_print(object_list_validated)

        return object_list_validated

    def save_video(self, name):
        # Does not work on MacOSX!
        pass

    def visualize(self, status=True):
        self.bool_visualisation = status
        print('Visualisation is: ', status)

    def set_timer(self, status=True):
        self.bool_timer = status
        print('Timer on')

    def validate(self, object_list):
        """
        things that aren't sure yet:
        should there be different validation methods and not only number of detection?
        How can we change the threshold? Should it be an input or a self.bool_threshold, etc? dictionary with different 
        threshold types? percentage, average, etc.
        """
        # This method takes a list of bottles and decides which ones are bottles and which ones aren't. It sets the ones
        # that aren't to status "False".
        start_time = time.time()

        return_list = []
        list_of_detections = get_list_info(object_list, "num_det")  # list with number of detection for each object
        # print('List of detections: ', list_of_detections)
        number_objects = len(object_list)
        #todo-cf: problem if no bottles are detected
        if number_objects == 0:
            return -1
        total_detections = sum(list_of_detections)
        # print('total detections: ', total_detections)

        # with this threshold the validation function decides what is an object and what is a false positive
        threshold = total_detections / number_objects

        for idx in range(len(list_of_detections)):
            if list_of_detections[idx] < threshold:
                # print('index ', idx, 'set to False')
                object_list[idx].set_status(False)
                # return_list.append(bottle_list[idx])
            elif list_of_detections[idx] >= threshold:
                # print('index ', idx, 'set to True')
                object_list[idx].set_status(True)
                return_list.append(object_list[idx])
        # print(get_list_info(bottle_list, "status"))

        print('validate time', time.time()-start_time)
        return return_list

    # def get_list_info(self, object_list, info):
    #     """
    #     This method gathers information about the objects of the class
    #     :param object_list:
    #     :param info:
    #     :return:
    #     """
    #     return_list = []
    #     for o in object_list:
    #         if info == 'num_det':
    #             return_list.append(o.get_num_det())
    #         elif info == 'detections':
    #             return_list.append(o.get_detections())
    #         elif info == 'mean':
    #             return_list.append(o.get_mean())
    #         elif info == 'status':
    #             return_list.append(o.get_status())
    #         elif info == 'name':
    #             return_list.append(o.get_name())
    #         elif info == 'angle':
    #             return_list.append(o.get_angle())
    #     return return_list

    def object_list_print(self, object_list):
        """
        Print this object_list in a nice way
        Maybe add input argument: printing style 
        :param object_list: 
        :return: 
        """

        # Find information to print
        name_list = (object_list, "name")
        angle_list = self.pixel2angle(object_list)
        num_det_list = get_list_info(object_list, "num_det")

        # Print
        for i in range(len(object_list)):
            # convert numpy to list
            angle_to_print = angle_list[i]
            angle_to_print = angle_to_print.tolist()
            print('%s has been detected %d times at a horizontal angle of %f' % (name_list[i],
                                                                                 num_det_list[i], angle_to_print[0]))

    def pixel2angle(self, object_list):
        """
        This method takes an object or a list of objects and converts each objects mean from pixels to angles and adds
        the correspondant angle to the objects
        :return: list with all angles
        """
        return_list = []
        if not isinstance(object_list, list):
            object_list = [object_list]

        for o in object_list:
            pixel = o.get_mean()
            pixel_tot = np.array(self.windowsize)
            # center is angle 0
            pixel_center = pixel_tot / 2
            pixel_new = pixel - pixel_center  # shifts origin to the center of the image
            angle_deg = (pixel_new / pixel_center) * self.fov_deg / 2
            return_list.append(angle_deg)
            print('angle_deg', angle_deg)
            # update bottle objects
            o.set_angle(angle_deg)

        return return_list

    def track(self):

        """
        How do we track? 
        1) Tracking only occurs when the robot moves, so we could use the information of the motor_control
        or the encoders to do the tracking. E.g. angle that has been done by robot --> compare object_list_validated 
        before and after movement
        2) Use OpenCV to track:
            input: old_object_list
            output object_list_tracked. 
            modify detect function: if self.bool_tracking == True: compare old bottle_list with new bottle_list

        What if the robot is moving while we're doing a detection? Should we forbid that?
        :return: 
        """
        # todo-cf: add track method!
        pass

    def truncate(self):
        """
        It chooses the ROI of the image where the bottles can be and applies the detect function only there!
        :return: 
        """
        # todo-cf: Implement truncate method
        pass

    def decision(self, object_list, type='angle_x'):
        """
        This method decides which bottle should be approached to grab it.
        :return: 
        """
        # todo-cf: finish decision method: 4 decision modes or mix of them: angle x, y, num det, w, h
        if type == 'num_det':
            num_det_list = get_list_info(object_list, "num_det")
            max_val = max(num_det_list)
            idx = num_det_list.index(max_val)
            return_object = object_list[idx]

        elif type == 'angle_x':
            angle_list = self.pixel2angle(object_list)
            angle_x_list = []
            for l in angle_list:
                angle_x_list.append(abs(l[0]))  # copy the absolute value of angle_x to new list
            min_val = min(angle_x_list)  # find minimum angle
            idx = angle_x_list.index(min_val)
            return_object = object_list[idx]  # return object with minimum angle_x

        elif type == 'angle_y':
            angle_list = self.pixel2angle(object_list)
            angle_y_list = []
            for l in angle_list:
                angle_y_list.append(l[1])  # copy the value of angle_y to new list
            min_val = min(angle_y_list)  # find minimum angle
            idx = angle_y_list.index(min_val)
            return_object = object_list[idx]  # return object with minimum angle_x

        elif type == 'angle_mix':
            pass

        elif type == 'num_det_angle_mix':
            # todo-cf: implement decision function mix: what is the function? what are the weights?
            pass

        elif type == 'rectangle':
            pass

        else:
            print('Error: Wrong type argument')
            return None

        print('The bottle to approach is: ', return_object.name)
        return return_object

def get_list_info(object_list, info):
    """
    This method gathers information about the objects of the class
    :param object_list: 
    :param info: 
    :return: 
    """
    return_list = []
    for o in object_list:
        if info == 'num_det':
            return_list.append(o.get_num_det())
        elif info == 'detections':
            return_list.append(o.get_detections())
        elif info == 'mean':
            return_list.append(o.get_mean())
        elif info == 'status':
            return_list.append(o.get_status())
        elif info == 'name':
            return_list.append(o.get_name())
        elif info == 'angle':
            return_list.append(o.get_angle())
    return return_list