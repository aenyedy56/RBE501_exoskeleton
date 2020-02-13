import csv
from dmp_experiments.Python.train_dmp import *
import numpy as np
from dmp_experiments.Python.DMP_runner import DMP_runner
import matplotlib.pyplot as plt
from decimal import *

# file = '/home/rhosea/catkin_ws/src/lidar_reading_package/sim_walking/leg_info.csv'

class LegsDMP(object):
    """
    Creates a person for accessing data and makings simplified plots
    """

    # leg_length is float length of leg in mm, foot_length is float length of foot in mm, height is float height of participant in mm
    def __init__(self):

        # file names of walking dmps
        self._lhip_dmp_walk = 'lhip_dmp_walk'
        self._lkne_dmp_walk = 'lkne_dmp_walk'
        self._lank_dmp_walk = 'lank_dmp_walk'
        self._rhip_dmp_walk = 'rhip_dmp_walk'
        self._rkne_dmp_walk = 'rkne_dmp_walk'
        self._rank_dmp_walk = 'rank_dmp_walk'
        # file names of stair climbing dmps
        self._lhip_dmp_stair = 'lhip_dmp_stair'
        self._lkne_dmp_stair = 'lkne_dmp_stair'
        self._lank_dmp_stair = 'lank_dmp_stair'
        self._rhip_dmp_stair = 'rhip_dmp_stair'
        self._rkne_dmp_stair = 'rkne_dmp_stair'
        self._rank_dmp_stair = 'rank_dmp_stair'

        # trained dmps for walking
        self.trained_lhip_walk = None
        self.trained_lkne_walk = None
        self.trained_lank_walk = None
        self.trained_rhip_walk = None
        self.trained_rkne_walk = None
        self.trained_rank_walk = None

        # trained dmps for stair climbing
        self.trained_lhip_stair = None
        self.trained_lkne_stair = None
        self.trained_lank_stair = None
        self.trained_rhip_stair = None
        self.trained_rkne_stair = None
        self.trained_rank_stair = None

        self.dt = None
        self.tau = None

    def plot_angle_trajectories(self,trajs):
        plt.title("2-D DMP demonstration")
        plt.xlabel("Time(t)")
        plt.ylabel("Angle Position(deg)")
        time = np.arange(0, self.tau + self.dt, self.dt)
        for i in range(0,len(trajs)):
            # plt.plot(time,des_trajs[i])
            plt.plot(time,trajs[i])
            plt._show()

    def move(self, steps, walk):
        groups =[]
        for i in range(0,len(steps)):
            if walk == 1:
                lhip_runner = DMP_runner(self._lhip_dmp_walk, steps[i][0][0], steps[i][0][1])
                lkne_runner = DMP_runner(self._lkne_dmp_walk, steps[i][1][0], steps[i][1][1])
                lank_runner = DMP_runner(self._lank_dmp_walk, steps[i][2][0], steps[i][2][1])
            elif walk == 0:
                lhip_runner = DMP_runner(self._lhip_dmp_stair, steps[i][0], steps[i+1][0])
                lkne_runner = DMP_runner(self._lkne_dmp_stair, steps[i][1], steps[i+1][1])
                lank_runner = DMP_runner(self._lank_dmp_stair, steps[i][2], steps[i+1][2])

            # dmp computed joint trajectories
            lhip_traj = []
            lkne_traj = []
            lank_traj = []
            rhip_traj = []
            rkne_traj = []
            rank_traj = []
            dmp_traj = []
            self.tau = 1
            for i in np.arange(0,int(self.tau/self.dt)+1):
                lhip_runner.step(self.tau, self.dt)
                lkne_runner.step(self.tau, self.dt)
                lank_runner.step(self.tau, self.dt)
                dmp_traj.append([lhip_runner.y,lkne_runner.y,lank_runner.y])
            groups.append(dmp_traj)
        #self.plot_angle_trajectories(dmp_traj)
        return groups


    def train_legs(self, csvname, num_basis_fcns, walk):

        # initialize variables
        T = []
        # joint positions
        q = []
        # joint velocities
        qd = []
        # joint accelerations
        qdd = []

        #open trial file
        self.dt = 0.002
        with open(csvname) as csvfile:
            reader = csv.reader(csvfile)
            #since each row is a joint trajectory we need those in q as the position, the velocity and accelerations are 0
            for row in reader:
                #reset each q for each joint
                q = []
                qd = [0]
                qdd = [0,0]
                for val in row:
                    q.append(float(val))
                qd = np.append(qd, np.divide(np.diff(q,1),np.power(self.dt,1)))
                qdd = np.append(qdd, np.divide(np.diff(q, 2), np.power(self.dt, 2)))
                T.append(q)
                T.append(qd)
                T.append(qdd)
            T_lhip = T[0:3]
            T_lkne = T[3:6]
            T_lank = T[6:9]
            # T_rhip = T[9:12]
            # T_rkne = T[12:15]
            # T_rank = T[15:18]

            trial_len = len(q)

        # self.dt = Decimal(1) / Decimal(trial_len)
        # self.dt = float(self.dt)

        # num_basis_fcns = 1000
        if walk == 1:
            self.trained_lhip_walk = train_dmp(self._lhip_dmp_walk,num_basis_fcns,T_lhip,self.dt)
            self.trained_lkne_walk = train_dmp(self._lkne_dmp_walk,num_basis_fcns,T_lkne,self.dt)
            self.trained_lank_walk = train_dmp(self._lank_dmp_walk,num_basis_fcns,T_lank,self.dt)
            # self.trained_rhip_walk = train_dmp(self._rhip_dmp_walk,num_basis_fcns,T_rhip,self.dt)
            # self.trained_rkne_walk = train_dmp(self._rkne_dmp_walk,num_basis_fcns,T_rkne,self.dt)
            # self.trained_rank_walk = train_dmp(self._rank_dmp_walk,num_basis_fcns,T_rank,self.dt)
        elif walk == 0:
            self.trained_lhip_stair = train_dmp(self._lhip_dmp_stair, num_basis_fcns, T_lhip, self.dt)
            self.trained_lkne_stair = train_dmp(self._lkne_dmp_stair, num_basis_fcns, T_lkne, self.dt)
            self.trained_lank_stair = train_dmp(self._lank_dmp_stair, num_basis_fcns, T_lank, self.dt)
            # self.trained_rhip_stair = train_dmp(self._rhip_dmp_stair, num_basis_fcns, T_rhip, self.dt)
            # self.trained_rkne_stair = train_dmp(self._rkne_dmp_stair, num_basis_fcns, T_rkne, self.dt)
            # self.trained_rank_stair = train_dmp(self._rank_dmp_stair, num_basis_fcns, T_rank, self.dt)


