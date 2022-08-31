# !/usr/bin/env python

### usage: 

# python3 evaluate_traj.py est.bag ref.bag /est_topic /ref_topic start_time end_time -v 

# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_traj.py \
# robot_localization_sync.bag usbl_accu5_rssi-50_inte100.bag \
# /rov/processed/robot_localization_odometry /usbl_odom \
# 1648583112.03  1648583439.4770002 -v

# python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/evaluate_traj.py \
# msckf_odom_sync.bag usbl_accu5_rssi-50_inte100.bag \
# /msckf/odom /usbl_odom \
# 1648583112.03  1648583439.4770002 -v


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
from evo.core import lie_algebra as lie

import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.family'] = 'serif'




class evaluateOdom:

    def __init__(self):
        self.bag_ref = ''
        self.bag_est = ''
        self.topic_ref = ''
        self.topic_est = ''
        self.start_time = 0
        self.end_time = 0

    def main(self):

      self.parse_arguments()

      self.align_traj()

      
    ### alignment for odom and grouth truth using given time duration 
    def align_traj(self):
        ### get ros message
        bag_handler_ref = rosbag.Bag(self.bag_ref)
        bag_handler_odom = rosbag.Bag(self.bag_est)
        self.traj_ref = file_interface.read_bag_trajectory(bag_handler_ref, self.topic_ref)
        self.traj_est = file_interface.read_bag_trajectory(bag_handler_odom, self.topic_est)

        ### check the given time duration
        s_ref = len(self.traj_ref.timestamps)
        s_odom = len(self.traj_est.timestamps)
        print('')
        print('given info:')
        print('ref: size=%s , start:%s, end:%s' % (s_ref, self.traj_ref.timestamps[0], self.traj_ref.timestamps[s_ref-1]) )
        print('odom: size=%s, start:%s, end:%s' % (s_odom, self.traj_est.timestamps[0], self.traj_est.timestamps[s_odom-1]))

        if (self.start_time >= self.traj_ref.timestamps[0]        and 
            self.start_time >= self.traj_est.timestamps[0]       and 
            self.end_time   <= self.traj_ref.timestamps[s_ref-1]  and 
            self.end_time   <= self.traj_est.timestamps[s_odom-1]):

            print('Given time duration is verified.')
        else:
            print('Error: given time duration is not right, ref:%s~%s, odom:%s~%s, given:%s~%s ' % 
                 (self.traj_ref.timestamps[0], self.traj_ref.timestamps[s_ref-1], 
                  self.traj_est.timestamps[0], self.traj_est.timestamps[s_odom-1],
                  self.start_time, self.end_time))

        ### select given time duration
        traj_ref_selected, traj_est_selected = self.select_duration()
        print('')
        print('selected info:')
        print('ref id=%s, size=%s' % (traj_ref_selected.meta["frame_id"], len(traj_ref_selected.timestamps)))
        print('odom id=%s, size=%s' % (traj_est_selected.meta["frame_id"], len(traj_est_selected.timestamps)))

        ### match the traj into same size 
        traj_ref_selected, traj_est_selected = sync.associate_trajectories(traj_ref_selected, traj_est_selected)

        ### align 
        r_a, t_a, s = traj_est_selected.align(traj_ref_selected)

        ### TEST for align entire trajectory
        traj_est_aligned = copy.deepcopy(self.traj_est)
        traj_est_aligned.transform(lie.se3(r_a, t_a))

        ### plot alignment result
        fig = plt.figure(1)
        plot_mode = plot.PlotMode.xyz
        ax = plot.prepare_axis(fig, plot_mode, subplot_arg=111)
        plot.traj(ax, plot_mode, traj_ref_selected, '--', 'gray')
        plot.traj(ax, plot_mode, traj_est_selected, '-', 'blue')
        fig.axes.append(ax)
        plt.title('partial alignment')

        fig = plt.figure(2)
        plot_mode = plot.PlotMode.xyz
        ax = plot.prepare_axis(fig, plot_mode, subplot_arg=111)
        plot.traj(ax, plot_mode, self.traj_ref, '--', 'gray')
        plot.traj(ax, plot_mode, traj_est_aligned, '-', 'blue')
        fig.axes.append(ax)
        plt.title('entire alignment')

        fig.tight_layout()
        plt.show()

    ### select duration of the trajectory based on given time duration
    def select_duration(self):

        ### select index of reference trajectory
        ref_start = -1
        ref_end = -1
        for i in range(len(self.traj_ref.timestamps)):    
            if self.traj_ref.timestamps[i] >= self.start_time:
                ref_start = i
                break
        for i in range(len(self.traj_ref.timestamps)):    
            if self.traj_ref.timestamps[i] >= self.end_time:
                ref_end = i
                break

        ### select index of odometry trajectory
        odom_start = -1
        odom_end = -1
        for i in range(len(self.traj_est.timestamps)):    
            if self.traj_est.timestamps[i] >= self.start_time:
                odom_start = i
                break
        for i in range(len(self.traj_est.timestamps)):    
            if self.traj_est.timestamps[i] >= self.end_time:
                odom_end = i
                break

        # print('debug odom: given=%s~%s, selected:%s~%s' %(self.start_time, self.end_time, 
        #         self.traj_est.timestamps[index_start], self.traj_est.timestamps[index_end]))        

        ### return the slected odom and ref trajectory
        return PoseTrajectory3D(np.array(self.traj_ref.positions_xyz[ref_start:ref_end]), 
                                np.array(self.traj_ref.orientations_quat_wxyz[ref_start:ref_end]), 
                                np.array(self.traj_ref.timestamps[ref_start:ref_end]),
                                meta={"frame_id": self.traj_ref.meta["frame_id"]}), \
               PoseTrajectory3D(np.array(self.traj_est.positions_xyz[odom_start:odom_end]), 
                                np.array(self.traj_est.orientations_quat_wxyz[odom_start:odom_end]), 
                                np.array(self.traj_est.timestamps[odom_start:odom_end]),
                                meta={"frame_id": self.traj_est.meta["frame_id"]})

    ### Parse arguments
    def parse_arguments(self):
        parser = argparse.ArgumentParser(description='evaluation for odometry and groundtruth.')
        parser.add_argument('bag_est', 
                            help='input bag contain odometry information')
        parser.add_argument('bag_ref', 
                            help='input bag contain reference(groundtruth) information') 
        parser.add_argument('topic_est', 
                            help='ros topic for estimation') 
        parser.add_argument('topic_ref', 
                            help='ros topic for reference') 
        parser.add_argument('start_time',type=float,
                            help='start time to select duration for alignment')
        parser.add_argument('end_time', type=float,
                            help='end time to select duration for alignment') 
        parser.add_argument('-v', '--verbose', action="store_true", default=False,
                            help='verbose output')


        args = parser.parse_args()

        if (args.verbose):
            print('')
            print('args info:')
            print("bag_est: %s" % args.bag_est)
            print("bag_ref: %s" % args.bag_ref)
            print("topic_est: %s" % args.topic_est)
            print("topic_ref: %s" % args.topic_ref)        
            print("start_time: %s" % args.start_time)
            print("end_time: %s" % args.end_time)

        ### set to global value
        self.bag_est = args.bag_est
        self.bag_ref = args.bag_ref
        self.topic_est = args.topic_est
        self.topic_ref = args.topic_ref
        self.start_time = args.start_time
        self.end_time = args.end_time

if __name__ == '__main__':
   object = evaluateOdom()
   object.main()