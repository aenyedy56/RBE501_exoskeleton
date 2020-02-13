from Vicon import Vicon
from tempfile import TemporaryFile
import numpy as np
import os
import csv
import matplotlib.pyplot as plt
from Vicon.Markers import *
import time
# 300 375
# 375 450
file_path = '/home/rhosea/catkin_ws/src/lidar_reading_package/sim_walking/subject_04_stair_config1_00.csv'
data = Vicon.Vicon(file_path)
model = data.get_model_output()
markers = data.get_markers()
markers.smart_sort()  # optional param to remove subject name

# get the angles of each joint
lhip_angles = model.get_left_leg().hip.angle.z
lkne_angles = model.get_left_leg().knee.angle.z
lank_angles = model.get_left_leg().ankle.angle.z
rhip_angles = model.get_right_leg().hip.angle.z
rkne_angles = model.get_right_leg().knee.angle.z
rank_angles = model.get_right_leg().ankle.angle.z

left_boundaries = [700, 800]
right_boundaries = [600, 700]

# angles converted from degrees to radians
left_angles = [lhip_angles[left_boundaries[0]:left_boundaries[1]],
          lkne_angles[left_boundaries[0]:left_boundaries[1]],
          lank_angles[left_boundaries[0]:left_boundaries[1]]]
right_angles = [rhip_angles[right_boundaries[0]:right_boundaries[1]],
          rkne_angles[right_boundaries[0]:right_boundaries[1]],
          rank_angles[right_boundaries[0]:right_boundaries[1]]]

for angle in left_angles:
    print(angle[0])


csvwriter = csv.writer(open('stair_00_00.csv', 'w'), delimiter=',', quotechar='|')
for x in left_angles:
    csvwriter.writerow(x)
for y in right_angles:
    csvwriter.writerow(y)
