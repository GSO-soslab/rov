# !/usr/bin/env python

# ------------------------------------------------------------------------------
# Function : save the image(compressed or raw) into video from rosbag
#
# Usage    : 
# python3 /home/lin/develop/ros/soslab_ws/src/rov/rov_remote/tools/scripts/py3/save_video.py \
# 40m_servo60_noLED_noAR_HighFeq10m.bag \
# /rov/sensors/stereo/left/image_raw/compressed left.avi \
# /rov/sensors/stereo/right/image_raw/compressed right.avi \
# -v
# ------------------------------------------------------------------------------

import roslib
import rosbag
import rospy
import argparse
import sys
import cv2
import numpy as np
import math

class SaveVideo:
    def __init__(self) :
        self.input_bag = ''
        self.left_topic = ''
        self.left_video_name = ''
        self.right_topic = ''
        self.right_video_name = ''

        self.height = 0
        self.width  = 0
        self.left_num = 0
        self.right_num = 0

    def main(self, argv):

        ############################################################################
        ####                          parse arguments                           ####
        ############################################################################
        parser = argparse.ArgumentParser(description='save the image(compressed or raw) into video from rosbag.')
        parser.add_argument('input_bag', 
                            help='input bag file')
        parser.add_argument('left_topic', 
                            help='the left topic of stereo cam') 
        parser.add_argument('left_video_name',
                            help='saved left video filename')
        parser.add_argument('right_topic', 
                            help='the right topic of stereo cam') 
        parser.add_argument('right_video_name',
                            help='saved right video filename')                            
        parser.add_argument('-v', '--verbose', action="store_true", default=False,
                            help='verbose output')

        args = parser.parse_args()

        if (args.verbose):
            print('')
            print("input bag: %s" % args.input_bag)
            print("left topic name: %s" % args.left_topic)
            print("left saved video: %s" % args.left_video_name)
            print("right topic name: %s" % args.right_topic)
            print("right saved video: %s" % args.right_video_name)

        ### set to global value
        self.input_bag = args.input_bag
        self.left_topic = args.left_topic
        self.left_video_name = args.left_video_name
        self.right_topic = args.right_topic
        self.right_video_name = args.right_video_name

        ############################################################################
        ####                           save video                               ####
        ############################################################################

        self.get_img_info()

        self.save_video(self.left_topic, self.left_video_name)
        
        self.save_video(self.right_topic, self.right_video_name)

    def save_video(self, ros_topic, filename):
        ### create saver
        saver = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'DIVX'), 
                                15, (self.width, self.height))

        ### start to save video
        print('')
        print('saving "%s" now ...' % filename)

        count = 0
        ratio = 0
        for topic, msg, t in rosbag.Bag(self.input_bag).read_messages(ros_topic):
            # convert ros compressed image to opencv format 
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # write to video
            saver.write(image_np)

            # check the percentage
            count +=1
            var = math.floor(count / self.left_num * 10)
            if var > ratio:
              ratio = var
              sys.stdout.write('%s' % ratio)
              sys.stdout.flush()

        # release saver
        saver.release()

        print('video saved!')

    def get_img_info(self):
        for topic, msg, t in rosbag.Bag(self.input_bag).read_messages(self.left_topic):
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self.height, self.width, layers = image_np.shape
            print('')
            print('Image info:')
            print(' height=%s, width=%s, layers=%s' %(self.height, self.width, layers))
            break
        
        self.left_num = rosbag.Bag(self.input_bag).get_message_count(self.left_topic)
        self.right_num = rosbag.Bag(self.input_bag).get_message_count(self.right_topic)
        print(' %s: has total %s messages' % (self.left_topic, self.left_num))
        print(' %s: has total %s messages' % (self.right_topic, self.right_num))

if __name__ == "__main__":
   object = SaveVideo()
   object.main(sys.argv[1:])