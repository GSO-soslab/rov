# !/usr/bin/env python

# ------------------------------------------------------------------------------
# Function : save the ROS odometry message into csv file 
#
# Usage    : 
# python3 /home/lin/develop/ros/soslab_ws/src/rov/rov_remote/tools/scripts/py3/odom_to_csv.py \
# example.bag /processed/odometry odometry.csv -v
# ------------------------------------------------------------------------------

import roslib
import rosbag
import rospy
import argparse
import sys
import numpy as np
from tf.transformations import euler_from_quaternion
import csv

class OdomToCSV:
    def __init__(self) :
        self.input_bag = ''
        self.topic = ''
        self.saved_file = ''

    def convert(self):
        print('')
        print('start to write csv file...')

        ### preapre the writer for csv file
        f = open(self.saved_file, 'w')
        writer = csv.writer(f)
        header = ['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
        writer.writerow(header)

        ### loop rosbag to get each odometry message
        for topic, msg, t in rosbag.Bag(self.input_bag).read_messages(self.topic):
            ### get t
            timestamp = msg.header.stamp.to_sec()
            ### get x,y,z
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            ### get roll,pitch,yaw
            q = msg.pose.pose.orientation
            q_arr = [q.x, q.y, q.z, q.w]
            (roll, pitch, yaw) = euler_from_quaternion (q_arr)

            ### write to csv
            data = [timestamp, x, y, z, roll, pitch, yaw]
            writer.writerow(data)

        ### close the csv writer
        f.close()

        print('Finsihed !!!')

    def main(self, argv):

        ############################################################################
        ####                          parse arguments                           ####
        ############################################################################
        parser = argparse.ArgumentParser(description='convert odometry into csv.')
        parser.add_argument('input_bag', 
                            help='input bag file')
        parser.add_argument('topic', 
                            help='the topic of odometry') 
        parser.add_argument('saved_file',
                            help='saved scv filename')
        parser.add_argument('-v', '--verbose', action="store_true", default=False,
                            help='verbose output')

        args = parser.parse_args()

        if (args.verbose):
            print('')
            print("input bag: %s" % args.input_bag)
            print("topic name: %s" % args.topic)
            print("saved filename: %s" % args.saved_file)

        ### set to global value
        self.input_bag = args.input_bag
        self.topic = args.topic
        self.saved_file = args.saved_file

        ############################################################################
        ####                          convert data                              ####
        ############################################################################

        self.convert()

if __name__ == "__main__":
   object = OdomToCSV()
   object.main(sys.argv[1:])