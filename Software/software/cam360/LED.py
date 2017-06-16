"""
This class creates LED - objects that are used for triangulation and therefore navigation. 
"""
import cv2
import numpy as np


class LED:
    def __init__(self, color):
        self.color = str(color)
        self.position = [None, None]  # one pixel which can be used to calculate angle!
        self.position_relative = [None, None]

        # position of beacon in arena
        if color == 'blue':
            self.position_arena = (8.0, 0)
            self.color_tuple = (226, 28, 28)
        elif color == 'yellow':
            self.position_arena = (0, 0)
            self.color_tuple = (28, 255, 255)
        elif color == 'red':
            self.position_arena = (8.0, 8.0)
            self.color_tuple = (31, 31, 239)
        elif color == 'green':
            self.position_arena = (0, 8.0)
            self.color_tuple = (31, 243, 31)
        else:
            self.position_arena = None
            self.color_tuple = (None, None, None)

        self.angle = None  # angle with reference to the y axis (from center of image towards bottom)
        self.mask = None  # color mask for led detection
        self.mask_processed = None
        self.contours = None
        self.contour_final = None  # final contour which corresponds to LED
        self.img = None
        self.img_color = None
        self.hierarchy = None
        self.moments = None
        self.moment_final = None  # final moment to calculate pixel position of LED
        self.edge = None


    def __repr__(self):
        return "%s LED" %(self.color)

    def __str__(self):
        return "%s LED" %(self.color)

    def get_angle(self):
        return self.angle
