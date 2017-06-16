import numpy as np
import cv2
import time
from random import randint
from importlib import import_module


# FUNCTIONS
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
            return_list.append(o.get_all_detections())
        elif info == 'mean':
            return_list.append(o.get_mean())
        elif info == 'status':
            return_list.append(o.get_status())
        elif info == 'name':
            return_list.append(o.get_name())
        elif info == 'angle':
            return_list.append(o.get_angle())
    return return_list

class HaarDetector:
    """
    This class starts a Haar detector, it takes the path to the classifier and the classname of the objects that want to
    be detected.
    """

    def __init__(self, path, classname):
        self.classifier = cv2.CascadeClassifier(str(path))
        self.classname = str(classname)

        # import the right class that corresponds to the classifier
        self.object_class = import_module(self.classname)
        self.class_dictionnary = {'bottle': self.object_class.Bottle}

        self.object_list = []

        # Field of View of camera, standard camera is PiCamera
        self.fov_deg = [36, 32]
        self.fov_rad = np.deg2rad(self.fov_deg)  # conversion to radians
        self.windowsize = np.array([])

        # Bool variables with their initial value
        self.bool_visualisation = False
        self.bool_timer = False
        self.bool_objects = True

        # frame
        self.frame = None

        # detection
        self.detection_list = []

        # global time counter to check age of detected bottles
        self.time_counter = 0

        # object counter
        self.object_counter = 0

        # total object counter
        self.object_counter_total = 0

        self.Y_ANGLE_THRESHOLD = 0.1

    def init_video(self):
        """
        This method starts the video capture.
        :rtype: list
        :return: list with the capture and information about it
        """
        self.cap = cv2.VideoCapture(0)
        self.windowsize = np.array([self.cap.get(3), self.cap.get(4)])
        return self.cap

    def get_frame(self, verbose=False, visualisation=False):
        self.ret, self.frame = self.cap.read()

        # todo-cf: add truncate method!!!

        if not self.ret:
            print("cap.read() didn't work properly")
            self.frame = None
        else:
            self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        return self.ret


    def new_object(self, inputlist):
        # todo-cf: new_object method: add obstacle possibility
        if self.classname == 'bottle':
            return_object = self.object_class.Bottle(inputlist[0], inputlist[1], inputlist[2], inputlist[3])
            self.object_counter_total = self.object_counter_total + 1
        elif self.classname == 'obstacle':
            """
            define object class and it's needed inputs. Put here so that objects can be detected!
            """
            pass
        else:
            print("Error: new_object couldn't be created, because ", self.classname, " is not correct")
            return_object = None
        return return_object

    def detect(self, verbose=False, visualisation=False):
        if verbose:
            print('\n Detect method: \n')

        detection_list = self.classifier.detectMultiScale(self.gray, 1.3, 5)

        self.object_counter = len(self.object_list)

        frame_counter = 0

        for (x, y, w, h) in detection_list:
            print('object_counter: ', self.object_counter)
            center = np.array([x + w / 2, y + h / 2])  # Calculate center of detection

            if len(self.object_list) == 0:
                if verbose:
                    print('Bottle list is empty')
                self.object_list.append(
                    self.new_object([center, self.windowsize, self.time_counter, '_'.join(['Bottle', str(self.object_counter_total)])]))
                self.object_counter = self.object_counter + 1
                self.object_list[0].detection_list.append([x, y, w, h])
                if verbose:
                    print('object list: ', self.object_list)
                    print('First bottle added\n')

            # If object_list not empty, see if new detection belongs to a existing object.
            else:
                distance_list = []  # store distances to existing objects to find shortest

                # Find closest bottle by analysing the shortest distance
                for b in self.object_list:
                    distance_list.append(b.get_dist(center))  #FIXME check if get_dist method is well done
                idx = np.argmin(distance_list)  # find index of smallest distance
                closest_bottle = self.object_list[idx]  # identify closest bottle

                # If it belongs to the closest object, append detection to object
                if closest_bottle.part_of_bottle(center):
                    closest_bottle.add_detection_manually(center, self.time_counter)
                    closest_bottle.detection_list.append([x, y, w, h])
                    self.object_list[idx] = closest_bottle
                    if visualisation:
                        cv2.putText(self.frame, closest_bottle.get_name(), (x, int(y - 0.03 * self.windowsize[1])),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255))

                # If it does not belong to an object, create a new one
                elif not closest_bottle.part_of_bottle(center):
                    self.object_list.append(
                        self.new_object([center, self.windowsize, self.time_counter, '_'.join(['Bottle', str(self.object_counter_total)])]))
                    self.object_list[-1].detection_list.append([x, y, w, h])
                    self.object_counter = self.object_counter + 1
                    if visualisation:
                        cv2.putText(self.frame, self.object_list[-1].get_name(), (x, int(y - 0.03 * self.windowsize[1])),
                                    cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 255))
                    print('Bottle added')
                    # drawing on image:
            if visualisation:
                cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

        return self.object_list

    def validate_bottle_age(self, verbose=False):
        if verbose:
            print('\n validate bottle age method:  \n')

        r = range(len(self.object_list))
        r_max = len(r) - 1  # max index
        print('r_max: ', r_max)

        for i, v in enumerate(reversed(r)):
            print('i: ', i)
            print('v: ', v)
            print(self.object_list)
            if self.object_list[r_max-i].check_age(self.time_counter) is True:
                 if verbose:
                     print(self.object_list[r_max-i].name, ' is young enough')
            else:
                 print(self.object_list)
                 if verbose:
                     print(self.object_list[r_max-i].name, ' is too old and therefore discarded')
                 self.object_list.pop(r_max-i)
                 print(self.object_list)

        return self.object_list

    def automated_detect(self, dtime=10):
        """
        Starts capture and detection function and validates them. Calculates the angles of the bottles and prints the
        whole information nicely
        :return: detection_list
        """
        # starts caption
        cap = self.init_video()
        # runs detection:
        object_list = self.detect(verbose=True, visualisation=True)
        # validates detected bottles
        status, object_list_validated = self.filter()
        # Print results nicely
        self.object_list_print(object_list_validated)

        return object_list_validated

    def filter(self, verbose=False, visualisation=False):
        """
        Filters out the detections by averaging over all detections and only considering those with more than average 
        """
        # This method takes a list of bottles and decides which ones are bottles and which ones aren't. It sets the ones
        # that aren't to status "False".
        if verbose:
            print('\n filter method:  \n')

        return_list = []
        list_of_detections = get_list_info(self.object_list, "num_det")  # list with number of detection for each object
        # print('List of detections: ', list_of_detections)
        number_objects = len(self.object_list)
        if number_objects == 0:
            if verbose:
                print('No bottles detected')
            return False, []

        # number of total detections
        total_detections = sum(list_of_detections)

        # with this threshold the validation function decides what is an object and what is a false positive
        threshold = total_detections / number_objects

        for idx in range(len(list_of_detections)):
            if list_of_detections[idx] < threshold:
                # print('index ', idx, 'set to False')
                self.object_list[idx].set_status(False)
                # return_list.append(bottle_list[idx])
            elif list_of_detections[idx] >= threshold:
                # print('index ', idx, 'set to True')
                self.object_list[idx].set_status(True)
                return_list.append(self.object_list[idx])
        if verbose:
            print(get_list_info(self.object_list, "status"))

        return True, return_list

    def object_list_print(self, object_list):
        """
        Print this object_list in a nice way
        Maybe add input argument: printing style 
        :param object_list: 
        :return: 
        """

        # Find information to print
        name_list = get_list_info(object_list, "name")
        angle_list = self.pixel2angle(object_list)
        num_det_list = get_list_info(object_list, "num_det")

        for i in range(len(object_list)):
            # convert numpy to list
            angle_to_print = angle_list[i]
            angle_to_print = angle_to_print.tolist()
            print('%s has been detected %d times at a horizontal angle of %f' % (name_list[i],
                                                                                 num_det_list[i], angle_to_print[0]))

    def filter_y_angle(self, verbose=True):
        if verbose:
            print('\n filter_y_angle: \n')
        for b in self.object_list:
            if b.angle[1] >= self.Y_ANGLE_THRESHOLD:
                if verbose:
                    print(b, 'deleted, because too large y_angle')
                del b


    def pixel2angle(self, object_list):
        """
        This method takes an object or a list of objects and converts each objects mean from pixels to angles and adds
        the correspondant angle to the objects
        :return: list with all angles
        """
        print('\n pixel2angle method: \n')
        return_list = []
        if not isinstance(object_list, list):
            object_list = [object_list]

        for o in object_list:
            pixel = o.get_mean()
            pixel_tot = np.array(self.windowsize)
            # center is angle 0
            pixel_center = pixel_tot / 2
            pixel_new = pixel - pixel_center  # shifts origin to the center of the image
            angle_rad = (pixel_new / pixel_center) * self.fov_rad / 2
            angle_rad[1] = -angle_rad[1]  # invert y-angle axis: standard coordinate system
            return_list.append(angle_rad)
            print('angle_rad', angle_rad)
            # update bottle objects
            o.set_angle(angle_rad)

        return return_list

    def visualize_buffer(self):

        print('\n visualize_buffer: \n')
        for b in self.object_list:
            print(b.vis_buffer[0])
            cv2.circle(self.frame, tuple(b.vis_buffer[0]), 5, b.tracking_color, -1)
            print('len b.vis_buffer: ', len(b.vis_buffer))
            for i in range(len(b.vis_buffer)):
                thickness = int(np.sqrt(b.BUFFER_SIZE / float(i + 1)) * 2.5)
                if i != 0:
                    cv2.line(self.frame, tuple(b.vis_buffer[i - 1]), tuple(b.vis_buffer[i]), b.tracking_color, thickness)

    def finalize(self, verbose=False, visualisation=False):
        self.cap.release()
        if visualisation:
            cv2.destroyAllWindows()
        if verbose:
            print('object list: ', self.object_list)
            print('detections lists: ', self.object_list_print(self.object_list))

    def decide(self, type='angle_x', verbose=True):
        """
        This method decides which bottle should be approached to grab it.
        :return: 
        """

        if verbose:
            print('\n decide method:  \n')

        if len(self.object_list) == 0:
            if verbose:
                print('Bottle list is empty: no bottle to track!')
            return False

        # if a bottle is being tracked, don't change it!
        if verbose:
            print('check if bottle is tracked already: \n')
        for b in self.object_list:
            if b.tracked:
                if verbose:
                    print(b, ' is being tracked already')
                return_object = b
                return return_object

        if verbose:
            print('no bottle is tracked, find new bottle to track: \n')
        if type == 'num_det':
            num_det_list = get_list_info(self.object_list, "num_det")
            max_val = max(num_det_list)
            idx = num_det_list.index(max_val)
            return_object = self.object_list[idx]
            self.object_list[idx].tracked = True

        elif type == 'angle_x':
            angle_list = self.pixel2angle(self.object_list)
            angle_x_list = []
            for l in angle_list:
                angle_x_list.append(abs(l[0]))  # copy the absolute value of angle_x to new list
            min_val = min(angle_x_list)  # find minimum angle
            idx = angle_x_list.index(min_val)
            return_object = self.object_list[idx]  # return object with minimum angle_x
            self.object_list[idx].tracked = True

        elif type == 'angle_y':
            angle_list = self.pixel2angle(self.object_list)
            angle_y_list = []
            for l in angle_list:
                angle_y_list.append(l[1])  # copy the value of angle_y to new list
            min_val = min(angle_y_list)  # find minimum angle
            idx = angle_y_list.index(min_val)
            return_object = self.object_list[idx]  # return object with minimum angle_x
            self.object_list[idx].tracked = True

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

def test():
    #classifier = "classifiers/haarcascade_frontalface_default.xml"
    #classifier = "classifiers/training2_0.xml"
    #classifier = "classifiers/final_training_8617.xml"
    classifier = "classifiers/final_training_7617.xml"
    bottle_detector = HaarDetector(classifier, 'bottle')
    bottle_detector.init_video()
    start_time = time.time()
    while time.time()-start_time<=1000:
        bottle_detector.time_counter = bottle_detector.time_counter + 1
        print('time counter: ', bottle_detector.time_counter)
        ret = bottle_detector.get_frame(verbose=True, visualisation=True)
        if ret is False:
            print('Cap not read properly')
            final_x_angle = -1
            final_y_angle = -1

        bottle_detector.detect(verbose=True, visualisation=False)
        bottle_detector.validate_bottle_age(verbose=True)
        bottle_detector.pixel2angle(bottle_detector.object_list)

        # # validate list, filter out false positives
        bottle_detector.filter(verbose=True)  # FIXME: needs to be done
        # # decide what bottle is pursued
        bottle_detector.filter_y_angle(verbose=True)

        bottle_detector.decide()  # FIXME: needs to be done


        for b in bottle_detector.object_list:
            print('\n angle: ', b.angle)

        # visualization:
        bottle_detector.visualize_buffer()
        bottle_detector.visualize()
        print(get_list_info(bottle_detector.object_list, "name"))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('quit successfully')
            break

    bottle_detector.finalize(visualisation=True, verbose=True)
