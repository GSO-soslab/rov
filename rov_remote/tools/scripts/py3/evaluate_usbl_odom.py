# !/usr/bin/env python

### usage: 
### the first infomration will be the reference frame in the alignment
# python3 evaluate_usbl_odom.py first.bag second.bag /first_topic /second_topic start_time end_time usbl_seq -v 

### end-align, msckf-usbl, 
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# msckf_odom_gyrobias.bag usbl_accu3_rssi-50_inte100.bag \
# /msckf/odom /usbl_odom 2 \
# 1648583112.03  1648583439.4770002 Ours -v

### end-align, rl-usbl, 
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# rl_odom_nofilter.bag usbl_accu3_rssi-50_inte100.bag \
# /rov/processed/robot_localization_odometry /usbl_odom 2 \
# 1648583112.03  1648583439.4770002 Robot_Localization -v

### begin-align, usbl-msckf,
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# usbl_accu3_rssi-50_inte100.bag msckf_odom_gyrobias.bag \
# /usbl_odom /msckf/odom 1 \
# 1648581892.29  1648581980.58 xy Ours -v

### begin-align, msckf-usbl,
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# msckf_odom_gyrobias.bag usbl_accu3_rssi-50_inte100.bag \
# /msckf/odom /usbl_odom 2 \
# 1648581892.29  1648581980.58 xy Ours -v

### begin-align, rl-usbl,
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# rl_odom_nofilter.bag usbl_accu3_rssi-50_inte100.bag \
# /rov/processed/robot_localization_odometry /usbl_odom 2 \
# 1648581892.29  1648581980.58 xy Robot_Localization -v

### begin-align, usbl-rl,
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# usbl_accu3_rssi-50_inte100.bag rl_odom_nofilter.bag \
# /usbl_odom /rov/processed/robot_localization_odometry 1 \
# 1648581892.29  1648581980.58 xy robot_localization -v

### entire-align, msckf-usbl,
# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_usbl_odom.py \
# msckf_odom_gyrobias.bag usbl_accu3_rssi-50_inte100.bag \
# /msckf/odom /usbl_odom 2 \
# 1648581892.29  1648583439.4770002 Ours -v

from curses import meta
from tokenize import Double
import numpy as np
import argparse
import sys
import rosbag
import copy

from evo.tools import file_interface, plot
from evo.core import sync
import evo.core.geometry as geometry
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.core import metrics
from evo.tools.settings import SETTINGS
from evo.core import lie_algebra as lie

import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.family'] = 'serif'

from sklearn.metrics import mean_squared_error



color_estimation = 'gray'
color_usbl = 'dodgerblue'
# plot_size = [12,8]
plot_size = [7,4]
# plot_size = figsize=SETTINGS.plot_figsize

class evaluateOdom:

    def __init__(self):
        self.bag_first = ''
        self.bag_second = ''
        self.topic_first = ''
        self.topic_second = ''
        self.start_time = 0
        self.end_time = 0
        self.usbl = -1
        self.est_name = ''

    def main(self):

      self.parse_arguments()

      self.align_traj()

    ### Parse arguments
    def parse_arguments(self):
        parser = argparse.ArgumentParser(description='evaluation for USBL and estimation odometry.')
        parser.add_argument('bag_first', 
                            help='input bag contain odometry information that used for reference frame')
        parser.add_argument('bag_second', 
                            help='input bag contain odometry information that used for child frame') 
        parser.add_argument('topic_first', 
                            help='ros topic from first bag') 
        parser.add_argument('topic_second', 
                            help='ros topic from second bag') 
        parser.add_argument('usbl', type=int,
                            help='which bag contain usbl data')                           
        parser.add_argument('start_time',type=float,
                            help='start time to select duration for alignment')
        parser.add_argument('end_time', type=float,
                            help='end time to select duration for alignment') 
        parser.add_argument('plot_mode', 
                            help='plot mode: xyz, xy, xz...')        
        parser.add_argument('est_name', 
                            help='the estimation name display in the plot')                                                
        parser.add_argument('-v', '--verbose', action="store_true", default=False,
                            help='verbose output')


        args = parser.parse_args()

        if (args.verbose):
            print('')
            print('args info:')
            print("bag_first: %s" % args.bag_first)
            print("bag_second: %s" % args.bag_second)
            print("topic_first: %s" % args.topic_first)
            print("topic_second: %s" % args.topic_second)        
            print("usbl: %s" % args.usbl)        
            print("start_time: %s" % args.start_time)
            print("end_time: %s" % args.end_time)
            print("plot_mode: %s" % args.plot_mode)
            print("est_name: %s" % args.est_name)

        ### set to global value
        self.bag_first = args.bag_first
        self.bag_second = args.bag_second
        self.topic_first = args.topic_first
        self.topic_second = args.topic_second
        self.usbl = args.usbl
        self.start_time = args.start_time
        self.end_time = args.end_time
        self.est_name = args.est_name

        if args.plot_mode == 'xyz':
            self.plot_mode = plot.PlotMode.xyz
        else:
            self.plot_mode = plot.PlotMode.xy

    ### align given trajectories
    def align_traj(self):

        ###################### handle rosbag ######################

        # get ros messages
        bag_handler_1 = rosbag.Bag(self.bag_first)
        bag_handler_2 = rosbag.Bag(self.bag_second)
        self.traj_1 = file_interface.read_bag_trajectory(bag_handler_1, self.topic_first)
        self.traj_2 = file_interface.read_bag_trajectory(bag_handler_2, self.topic_second)

        # check the given time duration
        size_1 = len(self.traj_1.timestamps)
        size_2 = len(self.traj_2.timestamps)
        print('')
        print('original data info:')
        print('first: size=%s , start:%s, end:%s' % 
            (size_1, self.traj_1.timestamps[0], self.traj_1.timestamps[size_1-1]) )
        print('second: size=%s, start:%s, end:%s' % 
            (size_2, self.traj_2.timestamps[0], self.traj_2.timestamps[size_2-1]))

        if (self.start_time >= self.traj_1.timestamps[0]        and 
            self.start_time >= self.traj_2.timestamps[0]        and 
            self.end_time   <= self.traj_1.timestamps[size_1-1] and 
            self.end_time   <= self.traj_2.timestamps[size_2-1]):

            print(' Given time duration is verified.')
        else:
            print('Error: given time duration is not right, first:%s~%s, second:%s~%s, given:%s~%s ' % 
                 (self.traj_1.timestamps[0], self.traj_1.timestamps[size_1-1], 
                  self.traj_2.timestamps[0], self.traj_2.timestamps[size_2-1],
                  self.start_time, self.end_time))
            sys.exit() 

        ###################### partial alignment ######################

        ### select given time duration
        alignment_traj_1, alignment_traj_2 = self.select_duration()

        ### match the traj into same size 
        alignment_traj_1, alignment_traj_2 = sync.associate_trajectories(alignment_traj_1, alignment_traj_2,0.1)

        print('')
        print('selected info:')
        print('traj 1 id=%s, size=%s' % (alignment_traj_1.meta["frame_id"], len(alignment_traj_1.timestamps)))
        print('traj 2 id=%s, size=%s' % (alignment_traj_2.meta["frame_id"], len(alignment_traj_2.timestamps)))

        ### get Umeyama alignment result 
        r_a, t_a, s = alignment_traj_2.align(alignment_traj_1)

        ### plot alignment result
        fig1 = plt.figure(figsize=plot_size, constrained_layout=True)
        ax = plot.prepare_axis(fig1, self.plot_mode, subplot_arg=111)
        if self.usbl ==1:
            plot.traj(ax, self.plot_mode, alignment_traj_1, '-', color_usbl, label='USBL')
            plot.traj(ax, self.plot_mode, alignment_traj_2, '-', color_estimation, label=self.est_name)
        else:
            plot.traj(ax, self.plot_mode, alignment_traj_1, '-', color_estimation, label=self.est_name)
            plot.traj(ax, self.plot_mode, alignment_traj_2, '-', color_usbl, label='USBL')
        ax.grid(linestyle='dashed', linewidth=1)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        fig1.axes.append(ax)
        ax.set_title('Alignment Plot')

        ###################### Entire Alignment ######################

        ### align entire trajectory
        traj_2_aligned = copy.deepcopy(self.traj_2)
        traj_2_aligned.transform(lie.se3(r_a, t_a))

        ### plot estimation and reference points

        # prepare plot
        fig2 = plt.figure(figsize=plot_size, constrained_layout=True)
        ax = plot.prepare_axis(fig2, self.plot_mode)

        if self.usbl == 1:
            # plot estimation at second seq
            plot.traj(ax, self.plot_mode, traj_2_aligned,
                    style='-',
                    color=color_estimation, label=self.est_name)
            # plot USBL
            if self.plot_mode == plot.PlotMode.xyz:
                ax.scatter(self.traj_1.positions_xyz[:, 0], 
                           self.traj_1.positions_xyz[:, 1], 
                           self.traj_1.positions_xyz[:, 2],
                           label='USBL', color=color_usbl)
            else:
                ax.scatter(self.traj_1.positions_xyz[:, 0], 
                           self.traj_1.positions_xyz[:, 1],
                           label='USBL', color=color_usbl)
        else:
            # plot estimation at first seq
            plot.traj(ax, self.plot_mode, self.traj_1,
                    style='-',
                    color=color_estimation, label=self.est_name)

            # plot USBL
            if self.plot_mode == plot.PlotMode.xyz:
                ax.scatter(traj_2_aligned.positions_xyz[:, 0], 
                           traj_2_aligned.positions_xyz[:, 1], 
                           traj_2_aligned.positions_xyz[:, 2],
                           label='USBL', color=color_usbl)
            else:
                ax.scatter(traj_2_aligned.positions_xyz[:, 0], 
                           traj_2_aligned.positions_xyz[:, 1],
                           label='USBL', color=color_usbl)

        ax.legend()

        ### plot correspondences for entire trajectory

        # get matched points
        pts_1_matched, pts_2_matched = sync.associate_trajectories(self.traj_1, traj_2_aligned, 0.1)
        print('')
        print('selected %s points for correspondences plot' % len(pts_1_matched.timestamps))

        # plot correspondences
        plot.draw_correspondence_edges(
                ax, pts_1_matched, pts_2_matched, self.plot_mode,
                style=SETTINGS.plot_pose_correspondences_linestyle,
                color='red',
                alpha=SETTINGS.plot_reference_alpha)

        # plot setting
        ax.grid(linestyle='dashed', linewidth=1)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        fig2.axes.append(ax)
        # ax.set_title('Correspondences Plot')
        fig2.savefig('/home/lin/Desktop/coor.png', format='png', dpi=300)

        ###################### plot error ######################

        # time 
        time=[]
        for i in range(len(pts_1_matched.timestamps)):
            time.append(pts_1_matched.timestamps[i] - pts_1_matched.timestamps[0])

        # error for x and y 
        dx = pts_1_matched.positions_xyz[:, 0] - pts_2_matched.positions_xyz[:, 0]
        dy = pts_1_matched.positions_xyz[:, 1] - pts_2_matched.positions_xyz[:, 1]

        # RMSE: USBL - estimation
        if self.usbl == 1:
            rmse_x = mean_squared_error(pts_1_matched.positions_xyz[:, 0], pts_2_matched.positions_xyz[:, 0], squared=False)
            rmse_y = mean_squared_error(pts_1_matched.positions_xyz[:, 1], pts_2_matched.positions_xyz[:, 1], squared=False)
        else:
            rmse_x = mean_squared_error(pts_2_matched.positions_xyz[:, 0], pts_1_matched.positions_xyz[:, 0], squared=False)
            rmse_y = mean_squared_error(pts_2_matched.positions_xyz[:, 1], pts_1_matched.positions_xyz[:, 1], squared=False)

        # # select data into 2 group: 1) used for alignment; 2) not used
        # used_dot = []
        # used_time = []
        # unused_dot = []
        # unused_time = []
        # for i in range(len(pts_1_matched.timestamps)):
        #     # inside alignment duration, set color to red
        #     if (pts_1_matched.timestamps[i] >= alignment_traj_1.timestamps[0] and
        #         pts_1_matched.timestamps[i] <= alignment_traj_1.timestamps[-1]):
        #         used_dot.append()
        #     # others set color to blue

        # Euclidean distance error
        # error_3d = []
        # for i in range(len(pts_1_matched.timestamps)):
        #     pt_1 = pts_1_matched.positions_xyz[i]
        #     pt_2 = pts_2_matched.positions_xyz[i]
        #     error_3d.append(np.linalg.norm(pt_1 - pt_2))

        # show color to separate 
        align_time_duration = alignment_traj_1.timestamps[-1] - alignment_traj_1.timestamps[0]
        col = np.where(time<align_time_duration,'r','b')


        # plot
        fig3, (ax1,ax2) = plt.subplots(nrows=2, ncols=1, figsize = plot_size, constrained_layout = True)
        # plot data
        scatter1 = ax1.scatter(time,dx,color=col,s=15, linewidth=0)
        scatter2 = ax2.scatter(time,dy,color=col,s=15, linewidth=0)
        # legend based on color
        # legend1 = ax1.legend(col, loc="lower left", title="Classes")
        # ax1.add_artist(legend1)
        # legend2 = ax2.legend(col, loc="lower left", title="Classes")
        # ax2.add_artist(legend2)
        # bin number of axis
        ax1.locator_params(nbins=6)
        ax2.locator_params(nbins=6)
        # grid line
        ax1.grid(linestyle='dashed', linewidth=1)
        ax2.grid(linestyle='dashed', linewidth=1)
        # show RMSE text
        ax1.text(0.2, 0.1, 'RMSE='+str(round(rmse_x,2)), horizontalalignment='center', 
                 verticalalignment='center', transform=ax1.transAxes)
        ax2.text(0.2, 0.1, 'RMSE='+str(round(rmse_y,2)), horizontalalignment='center', 
                 verticalalignment='center', transform=ax2.transAxes)
        # command axis
        ax1.set_title('X-axis')
        ax2.set_title('Y-axis')
        ax1.set_ylim([-15,10])
        ax2.set_ylim([-15,10])
        # common axis labels
        fig3.supxlabel('Time [s]')
        fig3.supylabel('Drift [m]')
        fig3.savefig('/home/lin/Desktop/error.png', format='png', dpi=300)

        ###################### Show plot ######################

        plt.show()


    ### select duration of the trajectory based on given time duration
    def select_duration(self):

        ### select index of trajectory 1
        traj_1_start = -1
        traj_1_end = -1
        for i in range(len(self.traj_1.timestamps)):    
            if self.traj_1.timestamps[i] >= self.start_time:
                traj_1_start = i
                break
        for i in range(len(self.traj_1.timestamps)):    
            if self.traj_1.timestamps[i] >= self.end_time:
                traj_1_end = i
                break

        ### select index of trajectory 2
        traj_2_start = -1
        traj_2_end = -1
        for i in range(len(self.traj_2.timestamps)):    
            if self.traj_2.timestamps[i] >= self.start_time:
                traj_2_start = i
                break
        for i in range(len(self.traj_2.timestamps)):    
            if self.traj_2.timestamps[i] >= self.end_time:
                traj_2_end = i
                break      

        ### return the slected trajectories
        return PoseTrajectory3D(np.array(self.traj_1.positions_xyz[traj_1_start:traj_1_end]), 
                                np.array(self.traj_1.orientations_quat_wxyz[traj_1_start:traj_1_end]), 
                                np.array(self.traj_1.timestamps[traj_1_start:traj_1_end]),
                                meta={"frame_id": self.traj_1.meta["frame_id"]}), \
               PoseTrajectory3D(np.array(self.traj_2.positions_xyz[traj_2_start:traj_2_end]), 
                                np.array(self.traj_2.orientations_quat_wxyz[traj_2_start:traj_2_end]), 
                                np.array(self.traj_2.timestamps[traj_2_start:traj_2_end]),
                                meta={"frame_id": self.traj_2.meta["frame_id"]})

if __name__ == '__main__':
   object = evaluateOdom()
   object.main()