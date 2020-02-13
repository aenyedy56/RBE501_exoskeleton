from Vicon import Vicon
from tempfile import TemporaryFile
import numpy as np
import os
import csv
import matplotlib.pyplot as plt
from Vicon.Markers import *
import time

def forward_kin(thi_len, shank_len, foot_len, q1, q2, q3, hip_pos):
    q1 = float(q1 * (math.pi) / 180)
    q2 = float(q2 * (math.pi) / 180)
    q3 = float(q3 * (math.pi) / 180)

    print(hip_pos[1])
    hip_pos = np.array([0, hip_pos[1], hip_pos[2]])
    knee_pos = np.array([0, (thi_len * cos(q1) + hip_pos[1]), -(thi_len * sin(q1) + hip_pos[2])])
    ankle_pos = np.array([0, (shank_len * cos(q1 + q2) + thi_len * cos(q1) + hip_pos[1]),
                          -(shank_len * sin(q1 + q2) + thi_len * sin(q1) + hip_pos[2])])
    toe_pos = np.array([0, (shank_len * cos(q1 + q2) + thi_len * cos(q1) - foot_len * sin(q1 + q2 + q3) + hip_pos[1]),
                        -(shank_len * sin(q1 + q2) + thi_len * sin(q1) + foot_len * sin(q1 + q2 + q3) + hip_pos[2])])
    xpos = np.array([hip_pos[1], knee_pos[1], ankle_pos[1], toe_pos[1]])
    ypos = np.array([hip_pos[2], knee_pos[2], ankle_pos[2], toe_pos[2]])
    return np.array([xpos, ypos])


def plot_leg(angles, hip_pos):
    plt.ion()
    fig = plt.figure()
    axes = fig.add_subplot(111)
    p, = plt.plot([], [], 'r-')
    plt.xlabel('x')
    plt.title('walk_one_leg')
    axes.set_xlim([-1000,2000])
    axes.set_ylim([-2500, 2000])
    for m in range(0, len(angles[0])):
        curr_hip_pos = np.array([0, hip_pos[1, m], hip_pos[2, m]])
        positions = forward_kin(300.0, 300.0, 125.0, angles[0][m], (angles[1][m]+90), angles[2][m]-90, curr_hip_pos)
        print(positions)
        p.set_data(positions[0],positions[1])
        fig.canvas.draw()

file_path = '/home/rhosea/catkin_ws/src/lidar_reading_package/sim_walking/subject_00 stairconfig1_00.csv'
data = Vicon.Vicon(file_path)
model = data.get_model_output()
markers = data.get_markers()
markers.smart_sort()  # optional param to remove subject name

# get positions of all the markers as tuples
lthi_frame = markers.get_rigid_body('LASI')
lkne_frame = markers.get_rigid_body('LKNE')
lank_frame = markers.get_rigid_body('LANK')
ltoe_frame = markers.get_rigid_body('LTOE')

rthi_frame = markers.get_rigid_body('RASI')
rkne_frame = markers.get_rigid_body('RKNE')
rank_frame = markers.get_rigid_body('RANK')
rtoe_frame = markers.get_rigid_body('RTOE')

#
lthi_traj = get_marker_traj(lthi_frame)
lkne_traj = get_marker_traj(lkne_frame)
ltoe_traj = get_marker_traj(ltoe_frame)
rthi_traj = get_marker_traj(rthi_frame)
rkne_traj = get_marker_traj(rkne_frame)
rtoe_traj = get_marker_traj(rtoe_frame)

# ltoe_traj = zero_calibrate_traj(ltoe_traj, ltoe_traj[0, 0], ltoe_traj[1, 0], ltoe_traj[2, 0])
# lkne_traj = zero_calibrate_traj(lkne_traj, ltoe_traj[0, 0], ltoe_traj[1, 0], ltoe_traj[2, 0])
# lhip_traj = zero_calibrate_traj(lthi_traj, ltoe_traj[0, 0], ltoe_traj[1, 0], ltoe_traj[2, 0])
# rtoe_traj = zero_calibrate_traj(rtoe_traj, rtoe_traj[0, 0], rtoe_traj[1, 0], rtoe_traj[2, 0])
# rkne_traj = zero_calibrate_traj(rkne_traj, rtoe_traj[0, 0], ltoe_traj[1, 0], ltoe_traj[2, 0])
# rhip_traj = zero_calibrate_traj(rthi_traj, rtoe_traj[0, 0], ltoe_traj[1, 0], ltoe_traj[2, 0])

# fig = plt.figure()
# plt.plot(lhip_traj[1], lhip_traj[2])
# plt.plot(rhip_traj[1], rhip_traj[2])
# plt.plot(lkne_traj[1], lkne_traj[2])
# plt.plot(rkne_traj[1], rkne_traj[2])
# plt.plot(ltoe_traj[1], ltoe_traj[2])
# plt.plot(rtoe_traj[1], rtoe_traj[2])
# plt.show()

lfemur_len = get_dist(lthi_frame[0][0], lkne_frame[0][0])
lshank_len = get_dist(lkne_frame[0][0], lank_frame[0][0])
lfoot_len = get_dist(lkne_frame[0][0], lank_frame[0][0])
rfemur_len = get_dist(rthi_frame[0][0], rkne_frame[0][0])
rshank_len = get_dist(rkne_frame[0][0], rank_frame[0][0])
rfoot_len = get_dist(rkne_frame[0][0], rank_frame[0][0])
leginfo = [lfemur_len, lshank_len, lfoot_len, rfemur_len, rshank_len, rfoot_len]
# print(leginfo)

left_hip_angles = model.get_left_leg().hip.angle.z
left_knee_angles = model.get_left_leg().knee.angle.z
left_ankle_angles = model.get_left_leg().ankle.angle.z
right_hip_angles = model.get_right_leg().hip.angle.x
right_knee_angles = model.get_right_leg().knee.angle.x
right_ankle_angles = model.get_right_leg().ankle.angle.x
left_angles = [left_hip_angles,
          left_knee_angles,
          left_ankle_angles]
right_angles = [right_hip_angles,
          right_knee_angles,
          right_ankle_angles]

# plot_leg(left_angles, lthi_traj)
csvwriter = csv.writer(open('leg_info.csv', 'w'), delimiter=',', quotechar='|')
ltoe_size = ltoe_traj.shape
# for m in range(0, ltoe_size[0]):
#     # for n in range(0, ltoe_size[1]):
#     csvwriter.writerow(ltoe_traj[m])


# for n in right_toe_traj:
#     csvwriter.writerow(n)

for x in left_angles:
    csvwriter.writerow(x)
for y in right_angles:
    csvwriter.writerow(y)
