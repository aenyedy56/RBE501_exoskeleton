import csv
from dmp_experiments.Python.train_dmp import *
import numpy as np
from dmp_experiments.Python.DMP_runner import DMP_runner
import matplotlib.pyplot as plt
from decimal import *

class DMP:

#data I used to make graph for presentation
# steps = [[-12,26,-19,-6,65,-35],[-12.09,13,-9,-5,62,-35]]

# lhip_pos = [-12,-12.09]
# lkne_pos = [26.3,18.4]
# lank_pos = [-19,-9]
# rhip_pos = [-6,-5]
# rkne_pos = [67,63]
# rank_pos = [-35,-35]


    def __init__(self, training_file):
        self._file = training_file


    def train(self):
        dt = 1
        trial_len = 0
        T = []
        q = []
        qd = []
        qdd = []
        with open(self._file) as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                q = []
                qd = [0]
                qdd = [0,0]
                for val in row:
                    q.append(float(val))
                qd = np.append(qd, np.divide(np.diff(q, 1), np.power(dt, 1)))
        	qdd = np.append(qdd, np.divide(np.diff(q, 2), np.power(dt, 2)))
                T.append(q)
                T.append(qd)
                T.append(qdd)
            T_lhip = T[0:3]
            T_lkne = T[3:6]
            T_lank = T[6:9]
            T_rhip = T[9:12]
            T_rkne = T[12:15]
            T_rank = T[15:18]
        
            trial_len = len(q)

        dt = Decimal(1) / Decimal(trial_len)
        self._dt = float(dt)

        self._lhip_dmp = 'lhip_dmp'
        self._lkne_dmp = 'lkne_dmp'
        self._lank_dmp = 'lank_dmp'
        self._rhip_dmp = 'rhip_dmp'
        self._rkne_dmp = 'rkne_dmp'
        self._rank_dmp = 'rank_dmp'

        num_basis_fcns = 1000
        self._trained_lhip = train_dmp(self._lhip_dmp,num_basis_fcns,T_lhip,self._dt)
        self._trained_lkne = train_dmp(self._lkne_dmp,num_basis_fcns,T_lkne,self._dt)
        self._trained_lank = train_dmp(self._lank_dmp,num_basis_fcns,T_lank,self._dt)
        self._trained_rhip = train_dmp(self._rhip_dmp,num_basis_fcns,T_rhip,self._dt)
        self._trained_rkne = train_dmp(self._rkne_dmp,num_basis_fcns,T_rkne,self._dt)
        self._trained_rank = train_dmp(self._rank_dmp,num_basis_fcns,T_rank,self._dt)


    def run(self, start_end_pos):
        # {lhips, lknee, lankle, rhips, rknee, rankle}
        # desired foot positions
        # joint_trajs = invKin(foot) # array of desired start and end angle positions for each joint
        # hardcoded joint hip start and end for now
        print(start_end_pos)
        lhip_pos = start_end_pos[0]
        lkne_pos = start_end_pos[1]
        lank_pos = start_end_pos[2]
        rhip_pos = start_end_pos[3]
        rkne_pos = start_end_pos[4]
        rank_pos = start_end_pos[5]


        lhip_runner = DMP_runner(self._lhip_dmp,lhip_pos[0],lhip_pos[1])
        lkne_runner = DMP_runner(self._lkne_dmp,lkne_pos[0],lkne_pos[1])
        lank_runner = DMP_runner(self._lank_dmp,lank_pos[0],lank_pos[1])
        rhip_runner = DMP_runner(self._rhip_dmp,rhip_pos[0],rhip_pos[1])
        rkne_runner = DMP_runner(self._rkne_dmp,rkne_pos[0],rkne_pos[1])
        rank_runner = DMP_runner(self._rank_dmp,rank_pos[0],rank_pos[1])

        Y = []
        tau = 1

        #{lhips, lknee, lankle, rhips, rknee, rankle}
        for i in np.arange(0,int(tau/self._dt)+1):
            lhip_runner.step(tau, self._dt)
            lkne_runner.step(tau, self._dt)
            lank_runner.step(tau, self._dt)
            rhip_runner.step(tau, self._dt)
            rkne_runner.step(tau, self._dt)
            rank_runner.step(tau, self._dt)
            Y.append((lhip_runner.y, lkne_runner.y, lank_runner.y, rhip_runner.y, rkne_runner.y, rank_runner.y))
        return Y

# plt.title("Left Knee DMP")
# plt.xlabel("Time(t)")
# plt.ylabel("Angle Position(deg)")
# print(Y)
# plt.plot(time[1:len(time)],T_lkne[0], label = 'Training Trajectory')
# plt.plot(time,Y, label = 'DMP Trajectory')
# plt.legend(loc="upper left")
# plt._show()
