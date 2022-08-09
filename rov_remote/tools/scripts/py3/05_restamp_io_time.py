# !/usr/bin/env python

# ------------------------------------------------------------------------------
# Function : restamp ros bagfile using IO time
#
# Usage    : 
# python3 /home/lin/develop/ros/soslab_ws/src/rov/rov_remote/tools/scripts/py3/05_restamp_io_time.py \
#  -i input.bag -o restamped.bag
# ------------------------------------------------------------------------------

import roslib
import rosbag
import rospy
import sys
import getopt
from   std_msgs.msg import String
from sensor_msgs.msg import TimeReference


class RemapTime:
    def __init__(self) :
        self.imu_calib = []
        self.mag_calib = []
        self.cam_left_calib = []
        self.cam_right_calib = []

    def main(self, argv):

        inputfile = ''
        outputfile = ''

        ############################################################################
        ####                          parse arguments                           ####
        ############################################################################
        try:
            opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
        except getopt.GetoptError:
            print ('usage: restamp_bag.py -i <inputfile> -o <outputfile>')
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                print ('usage: python restamp_bag.py -i <inputfile> -o <outputfile>')
                sys.exit()
            elif opt in ("-i", "--ifile"):
                inputfile = arg
            elif opt in ("-o", "--ofile"):
                outputfile = arg

        # print console header
        print ("")
        print ("restamp_bag")
        print ("")
        print ('input file:  ', inputfile)
        print ('output file: ', outputfile)
        print ("")
        print ("starting restamping (may take a while)")


        ############################################################################
        ####                            Get calib data                          ####
        ############################################################################
        print("")
        print("Starting to get calib data...")

        ## Get IMU calib data
        count = 0
        for topic, msg, t in rosbag.Bag(inputfile).read_messages(
            '/rov/sensors/ahrs/imu/calib'):
            self.imu_calib.append(msg)
            count +=1
        print(' finish grab IMU calib %s data' % count)

        ## Get Mag calib data
        self.mag_calib = self.imu_calib.copy()
        print(' finish grab Mag calib %s data' % count)

        ## Get Camera left calib data
        count = 0
        for topic, msg, t in rosbag.Bag(inputfile).read_messages(
            '/rov/sensors/stereo/right/calib'):
            count +=1
            self.cam_left_calib.append(msg)
        print(' finish grab cam left calib %s data' % count)
        ## Get Camera right calib data
        self.cam_right_calib = self.cam_left_calib.copy()
        print(' finish grab cam right calib %s data' % count)

        ############################################################################
        ####                             Write to bag                           ####
        ############################################################################
        print("")
        print("Starting to remap time...")

        outbag = rosbag.Bag(outputfile, 'w')

        io_stamp = rospy.Time()
        try:
            ###### replace for IMU data ######
            for topic, msg, t in rosbag.Bag(inputfile).read_messages(
                '/rov/sensors/ahrs/imu/data'):

                found,io_stamp = self.findImu(msg.header.stamp)

                if(found):
                    msg.header.stamp = io_stamp
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    print('imu data not found io time')
            print(' finish IMU IO time restamp')

            ###### replace for Mag data ######
            for topic, msg, t in rosbag.Bag(inputfile).read_messages(
                '/rov/sensors/ahrs/mag'):

                found,io_stamp = self.findMag(msg.header.stamp)

                if(found):
                    msg.header.stamp = io_stamp
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    print('mag data not found io time')
            print(' finish Mag IO time restamp')

            ###### replace for Stereo-Left ######
            for topic, msg, t in rosbag.Bag(inputfile).read_messages(
                '/rov_remote/sensors/stereo/left/image_raw_sync/compressed'):

                found,io_stamp = self.findCamLeft(msg.header.stamp)

                if(found):
                    msg.header.stamp = io_stamp
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    print('stereo-left data not found io time')
            print(' finish Stereo-left IO time restamp')

            ###### replace for Stereo-Right ######
            for topic, msg, t in rosbag.Bag(inputfile).read_messages(
                '/rov_remote/sensors/stereo/right/image_raw_sync/compressed'):

                found,io_stamp = self.findCamRight(msg.header.stamp)

                if(found):
                    msg.header.stamp = io_stamp
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    print('stereo-right data not found io time')
            print(' finish Stereo-right IO time restamp')

            ###### replace for DVL BT ######
            for topic, msg, t in rosbag.Bag(inputfile).read_messages(
                '/rov/sensors/dvl/df21/df21_sync'):
                msg.header.stamp = msg.ds_header.io_time
                outbag.write(topic, msg, msg.header.stamp)
            print(' finish DVL-BT IO time restamp')

            ###### replace for DVL CP ######
            for topic, msg, t in rosbag.Bag(inputfile).read_messages(
                '/rov/sensors/dvl/df3_sync'):
                msg.header.stamp = msg.ds_header.io_time
                outbag.write(topic, msg, msg.header.stamp)
            print(' finish DVL-CP IO time restamp')

        finally:
            print ("")
            print ("output bag written")
            outbag.close()

    # 1) based on current msg header timestamp, find the same one in calib header msg
    # 2) if found, assign the timestamp to the IO timestamp
    def findImu(self, sync_stamp):
        io_stamp = rospy.Time()

        for i in range(len(self.imu_calib)):
            if sync_stamp.to_sec() == self.imu_calib[i].header.stamp.to_sec() :
                # get io time
                io_stamp = self.imu_calib[i].time_ref
                # remove the old one
                self.imu_calib.remove(self.imu_calib[i])
                # return
                return True,io_stamp

        return False,io_stamp

    def findMag(self, sync_stamp):
        io_stamp = rospy.Time()

        for i in range(len(self.mag_calib)):
            if sync_stamp.to_sec() == self.mag_calib[i].header.stamp.to_sec() :
                # get io time
                io_stamp = self.mag_calib[i].time_ref
                # remove the old one
                self.mag_calib.remove(self.mag_calib[i])
                # return
                return True,io_stamp

        return False,io_stamp

    def findCamLeft(self, sync_stamp):
        io_stamp = rospy.Time()

        for i in range(len(self.cam_left_calib)):
            ## find by same sync time
            if sync_stamp.to_sec() == self.cam_left_calib[i].header.stamp.to_sec() :
                # get io time
                io_stamp = self.cam_left_calib[i].io_time
                # remove the old one
                self.cam_left_calib.remove(self.cam_left_calib[i])
                # return
                return True,io_stamp

        return False,io_stamp

    def findCamRight(self, sync_stamp):
        io_stamp = rospy.Time()

        for i in range(len(self.cam_right_calib)):
            ## find by same sync time
            if sync_stamp.to_sec() == self.cam_right_calib[i].header.stamp.to_sec() :
                # get io time
                io_stamp = self.cam_right_calib[i].io_time
                # remove the old one
                self.cam_right_calib.remove(self.cam_right_calib[i])
                # return
                return True,io_stamp

        return False,io_stamp

if __name__ == "__main__":
   remap_object = RemapTime()
   remap_object.main(sys.argv[1:])