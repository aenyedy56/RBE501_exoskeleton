#!/usr/bin/env python
import numpy as np
import math
from numpy import *
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class World(object):
	def __init__(self,l,h,r,w, legLength):
		# all units should be in mm
		# l - distance to s
		# h - stair height
		# r - stair run
		# w = stair width

		# these need to change later to incorporate the height data from the markers
		self._l = l
		self._h = h
		self._r = r
		self._w = w
		self._subj_leglength = legLength
		self._2d_fig = self.plot_2D_world()

		#self._3d_fig = self.plot_3D_world()

	def plot_2D_world(self):
		fig = plt.figure(1)

		#plt.plot([0,self._l,self._l,self._l+self._r*1,self._l+self._r*1,self._l+self._r*2,self._l+self._r*2,self._l+self._r*3],
		#		 [0,0,self._h*1,self._h*1,self._h*2,self._h*2,self._h*3,self._h*3])

		# plotting staircase using the vertical downward x axis and the horizontal y axis
		plt.plot([self._subj_leglength, self._subj_leglength, self._subj_leglength - self._h, self._subj_leglength - self._h],
				 [0, self._l, self._l, self._l+self._r])

		#plt.axis([0,self._l+self._r,0,self._h*1.2])
		plt.ion()
		plt.show()
		return fig

	def plot_3D_world(self):
		#need to incorporate width
		fig = plt.figure(2)
		plt.plot([0,self._l,self._l,self._l+self._r*1,self._l+self._r*1,self._l+self._r*2,self._l+self._r*2,self._l+self._r*3],
				 [0,0,self._h*1,self._h*1,self._h*2,self._h*2,self._h*3,self._h*3])
		#plt.axis([0,self._l+self._r,0,self._h*1.2])
		plt.ion()
		plt.show()
		return fig

"""
world = World()
floorL = 2
stepH = 0.133
stepR = 0.28
plot_world(floorL,stepH,stepR)
"""