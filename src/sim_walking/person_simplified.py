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
import copy

# contains methods for setting the foot, finding the trailing foot, and computing inverse kinematics on a given frame
# inverse kinematics assumes FOOT IS ON THE GROUND and not in an intermediate step (have to keep angle between ankle and floor 0-degrees)
# see inverse_kinematics_setup_diagram.png for image showing assumptions on angle locations/definitions 
class Person_simplified(object):
    """
    Creates a person for accessing data and makings simplified plots
    """

    # leg_length is float length of leg in mm, _foot_length is float length of foot in mm, height is float height of participant in mm
    def __init__(self, leg_length, foot_length):
        # biometric data
        self._leg_lenth = leg_length;
        self._thigh_length = float(leg_length*0.48)
        self._shank_length = float(leg_length*0.52)
        self._foot_length = foot_length  
        
        # global segment position data, using starting position of person as origin of world
        # each segment is an array of 3D coordinates, to be able to easily plot in 3D or in 2D
        self._left_hip_pos = [float(0.0), float(0.0), float(0.0)]
        self._left_knee_pos = [float(self._thigh_length),float(0.0), float(0.0)]
        self._left_ankle_pos = [float(self._thigh_length + self._shank_length), float(0.0), float(0.0)]
        self._left_toe_pos = [float(self._thigh_length + self._shank_length), float(0.0), float(self._foot_length)]

    def move_foot(self, distance_forward, distance_up):

        old_toe = copy.deepcopy(self._left_toe_pos)
        self._left_toe_pos = [self._left_toe_pos[0] - distance_up , self._left_toe_pos[1], self._left_toe_pos[2] + distance_forward]
        self._left_ankle_pos = [self._left_toe_pos[0], self._left_toe_pos[1], self._left_toe_pos[2]-self._foot_length]

        #move the toe to the kinematics workspace 
        toe = copy.deepcopy(self._left_toe_pos)
        toe[0] = toe[0] - distance_forward ;
        toe[2] = toe[2] - old_toe[2]
        
        if(distance_up > 0):
            print(toe)
            print(distance_up)
            print(distance_forward)

        jangles = self.inverse_kinematics2(toe)


        # forward kinematics using those jangles 
        self.forward_kinematics(jangles)
        return jangles


    def set_foot(self, foot, x, y, z):
        self._left_toe_pos = [x, y, z]
        self._left_ankle_pos = [x, y, z-self._foot_length]
       
    def step_down(self, distance):
        self._left_hip_pos[0] = self._left_hip_pos[0] + distance

    def inverse_kinematics2(self, toe):
        #print("IN IK2")
        # see inverse_kinematics_setup_diagram.png for visual of what each theta references
        # using equations from paper https://www.researchgate.net/publication/291955972_An_inverse_kinematic_algorithm_for_the_human_leg
        # ** specifically from section 3, using equations 3.1

        #print("PRE_OFFSET")
        #self.print_pos()
        #print("POST_OFFSET")
        # offset joint positions by hip position, to set hip at origin
        #self.subtract_hip_offset(left_hip_offset)

        #print(left_hip_offset)
        #self.print_pos()
        

        # calculating inverse kinematics for left leg:

        # Inverse kinematics using axis from the paper (and now matching our current axis system)
        lhip_angle = np.arctan((toe[2]-self._foot_length)/toe[0]) + np.arccos((self._thigh_length**2-self._shank_length**2+toe[0]**2+(toe[2]-self._foot_length)**2)/(2*self._thigh_length*np.sqrt(toe[0]**2 + (toe[2]-self._foot_length)**2)))
        lknee_angle = np.arccos((toe[0]**2+(toe[2]-self._foot_length)**2-self._thigh_length**2-self._shank_length**2)/(2*self._thigh_length*self._shank_length))
        lankle_angle = lhip_angle - lknee_angle + (np.pi/2)

        # reset joint positions to proper locations in world frame
        #self.add_hip_offset(left_hip_offset)

        # returning array of jangles in degrees
        jangles_array = [lhip_angle, lknee_angle, lankle_angle]
        jangles_array =np.degrees(jangles_array)
        #print("OUT IK")
        #print(jangles_array)
        return jangles_array

    def inverse_kinematics(self):
        print("IN IK")
        # see inverse_kinematics_setup_diagram.png for visual of what each theta references
        # using equations from paper https://www.researchgate.net/publication/291955972_An_inverse_kinematic_algorithm_for_the_human_leg
        # ** specifically from section 3, using equations 3.1

        # I am assuming that the coordinate system is using x-z for 2D plot instead of x-y, and z is the vertical axis in 3D plot 

        # the inverse kinematics equations assume that the hip is at the ORIGIN of the coordinate system, thus we must offset all points to place hip at origin
        # thus, we create an offset variable that takes the hip's global position in the simulation,
        # -> temporarily transforms all points for hip to be at origin, then put all points back to original coordinates
        left_hip_offset = copy.deepcopy(self._left_hip_pos)

        #print("PRE_OFFSET")
        #self.print_pos()
        #print("POST_OFFSET")
        # offset joint positions by hip position, to set hip at origin
        #self.subtract_hip_offset(left_hip_offset)

        #print(left_hip_offset)
        #self.print_pos()
        

        # calculating inverse kinematics for left leg:

        # Inverse kinematics using axis from the paper (and now matching our current axis system)
        lhip_angle = np.arctan((self._left_toe_pos[2]-self._foot_length)/self._left_toe_pos[0]) + np.arccos((self._thigh_length**2-self._shank_length**2+self._left_toe_pos[0]**2+(self._left_toe_pos[2]-self._foot_length)**2)/(2*self._thigh_length*np.sqrt(self._left_toe_pos[0]**2 + (self._left_toe_pos[2]-self._foot_length)**2)))
        lknee_angle = np.arccos((self._left_toe_pos[0]**2+(self._left_toe_pos[2]-self._foot_length)**2-self._thigh_length**2-self._shank_length**2)/(2*self._thigh_length*self._shank_length))
        lankle_angle = lhip_angle - lknee_angle + (np.pi/2)

        # reset joint positions to proper locations in world frame
        #self.add_hip_offset(left_hip_offset)

        # returning array of jangles in degrees
        jangles_array = [lhip_angle, lknee_angle, lankle_angle]
        jangles_array =np.degrees(jangles_array)
        #print("OUT IK")
        #print(jangles_array)
        return jangles_array

    def getTrailingFoot(self):
        return ('L', self._left_toe_pos)
        
    def forward_kinematics2(self, jangles, distance_forward, distance_up):
        #print(jangles)
        jangles = np.radians(jangles) # jangles is: {lhips, lknee, lankle, rhips, rknee, rankle}
        #self.print_pos()
    
        # assume left toe and ankle position are all set because they were already set in the set_foot method when setting new foot positions
        self._left_hip_pos = [self._left_hip_pos[0] - distance_up, self._left_hip_pos[1], self._left_hip_pos[2]+distance_forward]
        self._left_knee_pos = [round(self._left_hip_pos[0] + self._thigh_length*np.cos(jangles[0]),2), self._left_hip_pos[1], round(self._left_hip_pos[2] + self._thigh_length*np.sin(jangles[0]),2)]
        self._left_ankle_pos = [round(self._left_knee_pos[0] + round(self._shank_length*np.cos(jangles[0]-jangles[1]),2),2) , self._left_hip_pos[1], round(self._left_knee_pos[2] + round(self._shank_length*np.sin(jangles[0]-jangles[1]),2),2)]
        self._left_toe_pos = [self._left_ankle_pos[0] , self._left_ankle_pos[1], self._left_ankle_pos[2] + self._foot_length]
         

    # forward kinematics takes in jangles, basing calculations on root position 
    def forward_kinematics(self, jangles):
        #print("Forward Kin IN ")
        #print(jangles)
        # convert jangles to radians for trig operations
        jangles = np.radians(jangles) # jangles is: {lhips, lknee, lankle, rhips, rknee, rankle}
        #self.print_pos()
        # assume left toe and ankle position are all set because they were already set in the set_foot method when setting new foot positions
        self._left_knee_pos = [round(self._left_ankle_pos[0] - round(self._shank_length*np.cos(-np.pi/2+jangles[2]),2),2), self._left_ankle_pos[1], round(self._left_ankle_pos[2] - round(self._shank_length*np.sin(-np.pi/2+jangles[2]),2),2)]
        self._left_hip_pos = [round(self._left_knee_pos[0] - round(self._thigh_length*np.cos(-np.pi/2+jangles[2]+jangles[1]),2),2), self._left_ankle_pos[1], round(self._left_knee_pos[2] - round(self._thigh_length*np.sin(-np.pi/2+jangles[2]+jangles[1]),2),2)]
        
        #print("FK MID")
        #self.print_pos()
        #print("Forward Kin OUT ")

    def print_pos(self):
        print("Left Leg")
        print(str(self._left_hip_pos[0]) + ' ' + str(self._left_hip_pos[2]))
        print(str(self._left_knee_pos[0]) + ' ' + str(self._left_knee_pos[2]))
        print(str(self._left_ankle_pos[0]) + ' ' + str(self._left_ankle_pos[2]))
        print(str(self._left_toe_pos[0]) + ' ' + str(self._left_toe_pos[2]))

    # vector subtraction method for applying the hip offset: subtraction
    def subtract_hip_offset(self, left_hip_offset):
        # adding x, then y, then z elements of each position vector
        for i in range(3):
            self._left_hip_pos[i] = self._left_hip_pos[i] - left_hip_offset[i]
            self._left_knee_pos[i] = self._left_knee_pos[i] - left_hip_offset[i]
            self._left_ankle_pos[i] = self._left_ankle_pos[i] - left_hip_offset[i]
            self._left_toe_pos[i] = self._left_toe_pos[i] - left_hip_offset[i]


    # vector addition method for applying the hip offset: addition
    def add_hip_offset(self, left_hip_offset):
        # adding x, then y, then z elements of each position vector
        for i in range(3):
            self._left_hip_pos[i] = self._left_hip_pos[i] + left_hip_offset[i]
            self._left_knee_pos[i] = self._left_knee_pos[i] + left_hip_offset[i]
            self._left_ankle_pos[i] = self._left_ankle_pos[i] + left_hip_offset[i]
            self._left_toe_pos[i] = self._left_toe_pos[i] + left_hip_offset[i]

