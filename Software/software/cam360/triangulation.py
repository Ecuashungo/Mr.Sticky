import numpy as np
import math as m
import itertools as iter

class Triangulation:
    # todo-cf: add orientation
    # todo-cf: add robustness: compare with encoder info and old point that position calcultion is correct

    def __init__(self, led_list):
        self.led_list = led_list
        self.angle_list = []
        print(self.angle_list)
        self.color_list = []
        for led in self.led_list:
            self.angle_list.append(led.angle)
            self.color_list.append(led.color)

        self.beacon_dict = {'blue': (8.0, 0), 'yellow': (0, 0),
                            'green': (8.0, 8.0), 'red': (0, 8.0)}

        #self.total_led_list = []  # list that is used in the end for total algorithm

        self.robot_position = []

    def run(self):
        # if 3 led's available, calculate position of robot
        if len(self.led_list) == 3:
            print('3 leds given for trian.run()')
            self.robot_position = self.total(self.led_list)

        # if four led's available: calculate position 4 times and average
        elif len(self.led_list) == 4:
            print('4 leds given for triang.run()')
            # led_lists = self.total_led_list
            i1, i2, i3, i4 = iter.combinations(self.led_list, 3)
            led_lists = [i1, i2, i3, i4]

            robot_positions = []
            for ls in led_lists:
                robot_positions.append(self.total(ls))
            print('robot_positions: ', robot_positions)
            num_array = np.array(robot_positions)
            mean_array = np.mean(num_array, axis=0)
            print('mean calculated: ', mean_array)
            self.robot_position = mean_array.tolist()
        else:
            print('Wrong arguent given. Please give a ledlist with 3 or 4 leds')
            return False

        print("The robot's position is: ", self.robot_position)
        return self.robot_position

    def total(self, led_list):
        """
        This is the total algorithm for triangulation implemented. the angles in the led_list should be in radians.
        :param led_list: 
        :return: 
        """
        beacon_list_start = []
        angle_list = []
        # if led_list is tuple: conversion to list
        led_list = list(led_list)
        #print('led_list', led_list)

        # sorting list from smallest angle to largest.
        sorted_led_list = sorted(led_list, key=lambda x: x.angle, reverse=False)
        #print('sorted led_list', sorted_led_list)

        for led in sorted_led_list:
            beacon_list_start.append(self.beacon_dict[led.color])
            angle_list.append(led.angle)

        #print('angle list: ', angle_list)

        # Step 0: Definition of variables
        x1 = beacon_list_start[0][0]
        y1 = beacon_list_start[0][1]
        x2 = beacon_list_start[1][0]
        y2 = beacon_list_start[1][1]
        x3 = beacon_list_start[2][0]
        y3 = beacon_list_start[2][1]

        a1 = angle_list[0]
        a2 = angle_list[1]
        a3 = angle_list[2]

        # Step 1: compute modified beacon coordinates

        x1 = x1 - x2
        y1 = y1 - y2
        x3 = x3 - x2
        y3 = y3 - y2

        # Step 2: compute three cot()
        T12 = 1 / m.tan(a2 - a1)
        T23 = 1 / m.tan(a3 - a2)
        T31 = (1 - T12 * T23) / (T12 + T23)

        # Step 3: compute modified circle center coordinates
        x12 = x1 + T12 * y1
        y12 = y1 - T12 * x1
        x23 = x3 - T23 * y3
        y23 = y3 + T23 * x3
        x31 = (x3 + x1) + T31 * (y3 - y1)
        y31 = (y3 + y1) - T31 * (x3 - x1)

        # Step 4: compute k31
        k31 = x1 * x3 + y1 * y3 + T31 * (x1 * y3 - x3 * y1)

        # Step 5: compute D, if D=0 return error
        D = (x12 - x23) * (y23 - y31) - (y12 - y23) * (x23 - x31)
        if D == 0:
            print("error: in Total algorithm for triangulation: D = 0")
            raise ValueError("Total algorithm for triangulation: D = 0")
        # Step 6: compute robot position and return
        xr = x2 + (k31 * (y12 - y23)) / D
        yr = y2 + (k31 * (x23 - x12)) / D

        #todo-cf: test if theta works!
        theta = m.atan2((y2-yr),(x2-xr))-a2

        return [xr, yr, theta]

    # def orientation(self, led_list, pos):
    #     # if led_list is tuple: conversion to list
    #     led_list = list(led_list)
    #
    #     # sorting list from smallest angle to largest.
    #     sorted_led_list = sorted(led_list, key=lambda x: x.angle, reverse=False)
    #
    #     # 2 norm to green led:
    #     dy = (self.beacon_dict['green'][1] - pos[1])
    #     dx = (self.beacon_dict['green'][0] - pos[0])
    #
    #     if sorted_led_list[0].color == 'green':
    #         orientation = 2 * m.pi - sorted_led_list[-1].angle + m.tan(dy / dx)
    #     elif sorted_led_list[0].color == 'yellow':
    #         orientation = 2 * m.pi - sorted_led_list[-1].angle + m.tan(dy / dx)
