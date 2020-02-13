#!/usr/bin/env python
import numpy as np
#import math
from numpy import *
from math import sqrt
from math import sin
from math import cos
from sympy import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Person(object):
    """
    Creates a person for accessing data and makings simplified plots
    """

    # leg_length is float length of leg in mm, foot_length is float length of foot in mm, height is float height of participant in mm
    def __init__(self, leg_length, foot_length):
        self._thigh_length = leg_length*0.48
        self._shank_length = leg_length*0.52
        # self._foot_length = height*(25.4/167.64) # calculated using ratio from https://www.livestrong.com/article/491821-height-to-foot-size-ratio/ 
        self._foot_length = foot_length  
        self._left_leg_pos = []
        self._right_leg_pos = []
        q1 = symbols('q1')
        q2 = symbols('q2')
        q3 = symbols('q3')
        self._T01 = [[math.cos(q1), -1*math.sin(q1), 0, -1*self._foot_length*math.cos(q1)],
                     [math.sin(q1), math.cos(q1), 0, -1*self._foot_length*math.sin(q1)],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]
        self._T12 = [[math.cos(q2), -math.sin(q2), 0, -self._foot_length * math.cos(q2)],
                     [math.sin(q2), math.cos(q2), 0, -self._foot_length * math.sin(q2)],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]
        self._T23 = [[math.cos(q3), -math.sin(q3), 0, -self._foot_length * math.cos(q3)],
                     [math.sin(q3), math.cos(q3), 0, -self._foot_length * math.sin(q3)],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]
        self._T02 = self._T01*self._T12
        self._T03 = self._T02*self._T23


    @property
    def thigh_length(self):
        return self._thigh_length

    @property
    def shank_length(self):
        return self._shank_length

    @property
    def foot_length(self):
        return self._foot_length

    def get_stride(self):
        stride_2_height_ratio = 0.4 #needs to be calculated by looking at data
        return legLength*stride_2_height_ratio

    def steps_in_dist(self,dist):
        return floor(dist / self.get_stride())

    def joint_angle_2_joint_pos(self, angles):
        self.T01.subs({q1:angles(0)})
        self.T02.subs({q1: angles(0), q2: angles(1)})
        self.T03.subs({q1: angles(0), q2: angles(1), q3: angles(2)})
        # i think my dh might be reversed since I started at the foot
        # but I'll finish this after I do some dmp

    # assuming 3 axes in fig. First is world info like ground and stairs.
    # second is left leg and third is right leg.
    # dim is to update 2D vs 3D, 2 is 2D, 3 is 3D
    def update_leg_pos(self, fig, dim):
        if dim == 2:
            if len(fig.axes) == 1:
                fig.Axes.plot(self._left_leg_pos(0), self._left_leg_pos(0), 'ro',
                         self._right_leg_pos(0), self._right_leg_pos(0), 'blo')
            fig.axes(1).set_xdata(self._left_leg_pos(0)) #update x values for left leg
            fig.axes(1).set_ydata(self._left_leg_pos(1)) #update y values for left leg
            fig.axes(2).set_xdata(self._right_leg_pos(0)) #update y values for right leg
            fig.axes(2).set_xdata(self._left_leg_pos(1)) #update y values for right leg
        return fig
