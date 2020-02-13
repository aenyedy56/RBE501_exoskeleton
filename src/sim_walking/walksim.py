#!/usr/bin/env python
import numpy as np
import math
from numpy import *
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# import Person and World objects
from person_simplified import Person_simplified
from world import *
from LegsDMP import LegsDMP

#import Ros lib and custom message
import rospy
from lidar_reading_package.msg import Stairs


class WalkSim():

	def __init__(self):
		# using 9in, 7in, 12in, and 24in in mm for base staircase (based on Al-b's staircase)
		
		# using Al-b's biometric data for leg length and foot size
		self.subject = Person_simplified(1000, 279.4);
		self.dmp_runner = LegsDMP()
		self.dmp_runner.train_legs("walk_06_01_rightleg_halfstep.csv", 1000, 1);
		# using Al-b's biometric data for leg length and foot size

		self.plan = True;

		print("start position")
		self.subject.print_pos()
		jangles = self.subject.inverse_kinematics()
		print(jangles);
		self.subject.forward_kinematics(jangles)
		print("start position-after-ik-k")
		self.subject.print_pos()
		jangles = self.subject.inverse_kinematics()
		print(jangles)

		rospy.init_node('stair_path_planner')
		rospy.Subscriber("stairs", Stairs, self.walk)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()


	def walk(self, data):
		if(self.plan):
			self.plan = False
		else:
			return;

		print(data);
		stair_obj = data.stairs;
		# These should come from the message not be hard coded 
		# l - distance to s
		# h - stair height
		# r - stair run
		# w = stair width
		l = stair_obj[0].distance_to_stair
		h = stair_obj[0].height
		r = stair_obj[0].depth
		w = 609.6
		step_length = self.subject._leg_lenth * 0.413
		steps = 1
		# want to target getting within half a step of the stair so the next step takes us up the stair 
		dist_to_stair = l - (step_length/2)
		# if we are within a single step we are good
		if(dist_to_stair < 0):
			steps = 0
		else:
			step_length_remainder = dist_to_stair % step_length
			if (step_length_remainder == 0):
				# if their step magically lines up with the stairs don't do anything 
				#get the number of steps to get to the stairs based on their base stride 
				steps = floor(dist_to_stair / step_length)
			else:
				# add an additional step
				print(steps)
				steps = floor(dist_to_stair / step_length) + 1;
				print(steps)
				# and shorten the stride length by the remainder over the steps 
				step_length = step_length - ((step_length-step_length_remainder)/steps)
			

		walking_start_end_poses = []
		stair_climb_start_end_poses = []

		subject = Person_simplified(1000, 279.4)

		previous_angles = subject.inverse_kinematics()
		walking_start_end_poses = []
		for i in range(0, int(steps)):
			start_joint_angles = previous_angles

			goal_joint_angles = subject.move_foot(step_length, 0);

			joint_pairs = []
			for j in range(0, 3):
				joint_pairs.append((start_joint_angles[j], goal_joint_angles[j]))
			previous_angles = goal_joint_angles;

			walking_start_end_poses.append(joint_pairs);

			plt.figure(1)
			plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
			plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
			plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
			plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')
			plt.pause(0.01)

		stair_count = 1;
		for i in range(0, int(stair_count)):

			start_joint_angles = previous_angles
			goal_joint_angles = subject.move_foot(r*0.75, h);

			joint_pairs = []
			for j in range(0, 3):
				joint_pairs.append((start_joint_angles[j], goal_joint_angles[j]))
			previous_angles = goal_joint_angles;	

			stair_climb_start_end_poses.append(joint_pairs)

			plt.figure(1)
			plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
			plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
			plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
			plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')

		plt.show()
		print(walking_start_end_poses)
		print(stair_climb_start_end_poses)

		walk_angles = dmp_runner.move(walking_start_end_poses,1)

		stair_angles = dmp_runner.move(stair_climb_start_end_poses,1)

		subject2 = Person_simplified(1000, 279.4)
		staircase = World(l, h, r, w, subject2._leg_lenth)
		i = 0;
		print(len(walk_angles))
		for i in walk_angles:
			print("step " + str(len(i)))
			jangles = []
			for j in i:
				#disp(j)
				subject2.forward_kinematics2(j, (step_length/len(i)), 0);
				plt.figure(1) 
				plt.plot(subject2._left_hip_pos[0], subject2._left_hip_pos[2], 'ro')
				plt.plot(subject2._left_knee_pos[0], subject2._left_knee_pos[2], 'go')
				plt.plot(subject2._left_ankle_pos[0], subject2._left_ankle_pos[2], 'bo')
				plt.plot(subject2._left_toe_pos[0], subject2._left_toe_pos[2], 'mo')
				jangles = j
			# The starting position is stainding striaght up, and since there is no back leg
			# the leg doesn't come back down which requires the back hip rotating 
			# soe we drop it artificially 
			distance_down = 1000 - subject2._left_toe_pos[0]
			for i in range(0, 50):
				plt.figure(1) 
				subject2.step_down(distance_down/50)
				subject2.forward_kinematics2(j, 0,0);
				plt.plot(subject2._left_hip_pos[0], subject2._left_hip_pos[2], 'ro')
				plt.plot(subject2._left_knee_pos[0], subject2._left_knee_pos[2], 'go')
				plt.plot(subject2._left_ankle_pos[0], subject2._left_ankle_pos[2], 'bo')
				plt.plot(subject2._left_toe_pos[0], subject2._left_toe_pos[2], 'mo')
			plt.show()
			plt.pause(0.1)

		print(len(stair_angles))
		for i in stair_angles:
			print("step " + str(len(i)))
			for j in i:
				#disp(j)
				subject2.forward_kinematics2(j, ((r*0.75)/len(i)), (h/(len(i))));
				plt.figure(1) 
				plt.plot(subject2._left_hip_pos[0], subject2._left_hip_pos[2], 'ro')
				plt.plot(subject2._left_knee_pos[0], subject2._left_knee_pos[2], 'go')
				plt.plot(subject2._left_ankle_pos[0], subject2._left_ankle_pos[2], 'bo')
				plt.plot(subject2._left_toe_pos[0], subject2._left_toe_pos[2], 'mo')
			plt.show()
			plt.pause(0.1)	
			

if __name__ == '__main__':
	try:
		ne = WalkSim()
	except rospy.ROSInterruptException: pass	