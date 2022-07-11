# !/usr/bin/env python


#------------------------------------------------------------------------------
# This script used to remap DVL restore time because of the DVL trigger issue
#------------------------------------------------------------------------------

from operator import truediv
import roslib
import rosbag
import rospy
import sys
from   std_msgs.msg import String
from nortek_dvl.msg import ButtomTrack
from ds_sensor_msgs.msg import NortekDF21
import argparse
import matplotlib.pyplot as plt
import pickle

# example:
# python3 /home/lin/develop/ros/soslab_ws/src/rov/rov_remote/tools/scripts/py3/02_recovery_sync_dvl.py \
# raw_total.bag out_test.bag \
# /rov/sensors/dvl/df21 /rov/sensors/dvl/df21/df21_sync \
# /rov/sensors/dvl/df3 /rov/sensors/dvl/df3/df3_sync  -v

def main():

    #-------------------------------- Get arguments ---------------------------------------------#

    # get arguments
    parser = argparse.ArgumentParser(description='Restore DVL synchronized time')
    parser.add_argument('raw_bag',
                        help='input bag file')
    parser.add_argument('correct_bag',
                        help='output bag file')                      
    parser.add_argument('topic_bt_raw', 
                        help='topic for raw DVL BT data')
    parser.add_argument('topic_bt_sync', 
                        help='topic for time synchronized DVL BT data')
    parser.add_argument('topic_cp_raw', 
                        help='topic for raw DVL CP data')
    parser.add_argument('topic_cp_sync', 
                        help='topic for time synchronized DVL CP data')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    args = parser.parse_args()

    # print arguments

    if (args.verbose):
        print("Input rosbag is: " + args.raw_bag)
        print("Input topic_bt_raw is: " + args.topic_bt_raw)
        print("Input topic_bt_sync is: " + args.topic_bt_sync)
        print("Input topic_cp_raw is: " + args.topic_cp_raw)
        print("Input topic_cp_sync is: " + args.topic_cp_sync)
        print("Output rosbag is: " + args.correct_bag)

    input_bag = rosbag.Bag(args.raw_bag)
    output_bag = rosbag.Bag(args.correct_bag, 'w')

    #------------------------------------------ BT Align  ----------------------------------------------------#

    #### select raw or sync, which one assign as the first: the larger timestamp
    first_bt_raw_sys_t = 0
    first_bt_sync_sys_t = 0
    # find first raw DVL BT data
    for topic, msg, t in input_bag.read_messages(args.topic_bt_raw):
        first_bt_raw_sys_t = msg.dvl_time
        print('first raw BT system time:' , first_bt_raw_sys_t)
        break
    # find first sync DVL BT data
    for topic, msg, t in input_bag.read_messages(args.topic_bt_sync):
      first_bt_sync_sys_t = msg.dvl_time
      print('first sync BT system time:' , first_bt_sync_sys_t)
      break   


    first_bt_trigger_t = 0
    first_bt_raw_t = 0

    ### select based on raw
    if first_bt_raw_sys_t > first_bt_sync_sys_t:
      ### find first raw time
      first_bt_raw_t = first_bt_raw_sys_t

      ### find first trigger time
      for topic, msg, t in input_bag.read_messages(args.topic_bt_sync):
        if first_bt_raw_sys_t == msg.dvl_time:
          # restore to original trigger time
          time_transmit = (msg.timeDiff1Beam[0] + msg.timeDiff1Beam[1] +
                          msg.timeDiff1Beam[2] + msg.timeDiff1Beam[3]) / 4;
          first_bt_trigger_t = msg.header.stamp.to_sec() - 0.0084 - time_transmit
          print('align sync BT 1 sync time:%f with system time:%f' %(first_bt_trigger_t, msg.dvl_time))
          break
        else:
          print('something wrong1!')
    ### selec based on sync
    else :
      ### find first raw time
      for topic, msg, t in input_bag.read_messages(args.topic_bt_raw):
        if first_bt_sync_sys_t == msg.dvl_time:
          first_bt_raw_t = msg.dvl_time
          break

      ### find first trigger time
      for topic, msg, t in input_bag.read_messages(args.topic_bt_sync):
        if first_bt_sync_sys_t == msg.dvl_time:
          # restore to original trigger time
          time_transmit = (msg.timeDiff1Beam[0] + msg.timeDiff1Beam[1] +
                          msg.timeDiff1Beam[2] + msg.timeDiff1Beam[3]) / 4;
          first_bt_trigger_t = msg.header.stamp.to_sec() - 0.0084 - time_transmit
          print('align sync BT 2 sync time:%f with system time:%f' %(first_bt_trigger_t, msg.dvl_time))
          break
        else:
          print('something wrong3!')

    print('Finish aligning raw data and sync data for BT !!')

    #------------------------------------------ CP Align  ----------------------------------------------------#
    first_cp_raw_sys_t = 0
    first_cp_trigger_t = 0
    # find first raw DVL CP data
    for topic, msg, t in input_bag.read_messages(args.topic_cp_raw):
        first_cp_raw_sys_t = msg.dvl_time
        print('first raw CP system time:' , first_cp_raw_sys_t)
        break
    # find first sync DVL CP data
    for topic, msg, t in input_bag.read_messages('/rov/sensors/dvl/df3_sync'):
      if first_cp_raw_sys_t == msg.dvl_time:
        first_cp_trigger_t = msg.header.stamp.to_sec() - 0.0084 
        print('firsy sync CP sync time:%f with system time:%f' %(first_cp_trigger_t, msg.dvl_time))
        break
    print('Finish aligning raw data and sync data for CP !!')

    #--------------------------------------- BT recovery  -------------------------------------------#
    # recovery synchronized DVL BT time
    # TODO: correct DVL time into arduino time, apply a ratio for all time measurements in DVL 
    bt_dvl_raw = []
    bt_dvl_sync = []
    last_sys_t = 0
    trigger_delay = 0.0084
    drift = 0.000013333 # arduino drift

    for topic, msg, t in input_bag.read_messages(args.topic_bt_raw):
      # only start at the align point
      if first_bt_raw_t > msg.dvl_time:
        continue

      # actual sync time should be: sync time = trigger time + trigger delay + half transmit time

      # get correctted trigger time
      if len(bt_dvl_sync) == 0:
        correct_trigger_t = first_bt_trigger_t
      else:
        # find how many trigger missed, apply the arduino trigger time
        delta_sync_time = round( (msg.dvl_time - last_sys_t) / 0.125) * (0.125+drift)
        correct_trigger_t += delta_sync_time

      # get half transmit time from this message
      time_transmit = (msg.timeDiff1Beam[0] + msg.timeDiff1Beam[1] +
                       msg.timeDiff1Beam[2] + msg.timeDiff1Beam[3]) / 4;

      # get correct BT time
      time = correct_trigger_t + trigger_delay + time_transmit
      last_sys_t = msg.dvl_time      
      
      # correct BT msg and write to rosbag
      sync_msg = msg
      sync_msg.header.stamp = rospy.Time.from_sec(time)
      output_bag.write(args.topic_bt_sync, sync_msg, sync_msg.header.stamp)

      # append sync BT time for visualization
      bt_dvl_sync.append(sync_msg)
      # append raw BT time for visualization
      bt_dvl_raw.append(msg)

    print('Finish recovering sync data for BT and saved rosbag with total:%d recoveried msg!!' % len(bt_dvl_sync))

    # # TEST:411.7474s, drift 0.0439198 -> each ping(0.125s) drift 0.000013333s
    # test_delta = []
    # test_seq = []
    # for topic, msg, t in input_bag.read_messages(args.topic_bt_sync):
    #   # find recoveried synchronized DVL BT data, by align the DVL system time
    #   for index, item in enumerate(bt_dvl_sync):
    #     if item.dvl_time == msg.dvl_time:
    #         break
    #     else:
    #         index = -1
    #   if index != -1:
    #     # get timeoffset
    #     delta_t = bt_dvl_sync[index].header.stamp.to_sec() - msg.header.stamp.to_sec()
    #     test_delta.append(delta_t)
    #     test_seq.append(bt_dvl_sync[index].header.stamp.to_sec())
    # print("FINSIH TEST with total: %d" % len(test_delta))


    #-------------------------------- BT plot data ---------------------------------------------#

    bt_delta_raw_sys_t = []
    bt_delta_sync_t = []
    bt_seq = []
    for i in range(1, len(bt_dvl_raw)):
        delta = bt_dvl_raw[i].dvl_time - bt_dvl_raw[i-1].dvl_time
        bt_delta_raw_sys_t.append(delta)

        delta = bt_dvl_sync[i].dvl_time  - bt_dvl_sync[i-1].dvl_time
        bt_delta_sync_t.append(delta)
        bt_seq.append(bt_dvl_raw[i].header.seq)

    print('Finish preparing plot data for BT !!')

    #------------------------------ CP recovery  -------------------------------------#
    # recovery synchronized DVL CP time
    cp_dvl_raw = []
    cp_sync_t = []
    last_sys_t = 0

    for topic, msg, t in input_bag.read_messages(args.topic_cp_raw):
      # actual sync time should be: sync time = trigger time + trigger delay 

      # get correctted trigger time
      if len(cp_sync_t) == 0:
        correct_trigger_t = first_cp_trigger_t
      else:
        # find how many trigger missed, apply the arduino trigger time

        ### special case: DVL terigger time not trigger at 1 constant second, but 0.875
        if msg.header.seq == 892:
          delta_sync_time = round( (msg.dvl_time - last_sys_t) / 1.0) * (0.875+8*drift)
        else:
          delta_sync_time = round( (msg.dvl_time - last_sys_t) / 1.0) * (1.0+8*drift)

        ### Normal case: trigger every 1 second, 
        # delta_sync_time = round( (msg.dvl_time - last_sys_t) / 1.0) * (1.0+8*drift)

        correct_trigger_t += delta_sync_time
        # if msg.header.seq > 890 and msg.header.seq < 895:
        #   test = (msg.dvl_time - last_sys_t) 
        #   print(test)

      # get correct BT time
      time = correct_trigger_t + trigger_delay
      last_sys_t = msg.dvl_time      
      
      # correct BT msg and write to rosbag
      sync_msg = msg
      sync_msg.header.stamp = rospy.Time.from_sec(time)
      output_bag.write(args.topic_cp_sync, sync_msg, sync_msg.header.stamp)

      # append sync BT time for visualization
      cp_sync_t.append(time)
      # append raw BT time for visualization
      cp_dvl_raw.append(msg)

    print('Finish recovering sync data for CP and saved rosbag with total:%d recoveried msg!!' % len(cp_sync_t))

    #-------------------------------- CP plot data ---------------------------------------------#
    cp_delta_raw_sys_t = []
    cp_delta_sync_t = []
    cp_seq = []
    for i in range(1, len(cp_dvl_raw)):
        delta = cp_dvl_raw[i].dvl_time - cp_dvl_raw[i-1].dvl_time
        cp_delta_raw_sys_t.append(delta)

        delta = cp_sync_t[i] - cp_sync_t[i-1]
        cp_delta_sync_t.append(delta)
        
        cp_seq.append(cp_dvl_raw[i].header.seq)

    print('Finish preparing plot data for CP !!')

    #-------------------------------- Plot ---------------------------------------------#
    output_bag.close()

    fig_1 = plt.figure(1)
    ax_1 = plt.subplot(1, 1, 1)
    ax_1.scatter(bt_seq, bt_delta_raw_sys_t)
    plt.xlabel('data seq')
    plt.ylabel('system time difference (s)')
    plt.title('DVL BT system time offset on raw data')
    plt.grid(True)

    fig_2 = plt.figure(2)
    ax_2 = plt.subplot(1, 1, 1)
    ax_2.scatter(bt_seq, bt_delta_sync_t)
    plt.xlabel('data seq')
    plt.ylabel('sync time difference (s)')
    plt.title('DVL BT system time offset on sync data')
    plt.grid(True)

    fig_3 = plt.figure(3)
    ax_3 = plt.subplot(1, 1, 1)
    ax_3.scatter(cp_seq, cp_delta_raw_sys_t)
    plt.xlabel('data seq')
    plt.ylabel('system time difference (s)')
    plt.title('DVL CP system time offset on raw data')
    plt.grid(True)

    fig_4 = plt.figure(4)
    ax_4 = plt.subplot(1, 1, 1)
    ax_4.scatter(cp_seq, cp_delta_sync_t)
    plt.xlabel('data seq')
    plt.ylabel('sync time difference (s)')
    plt.title('DVL CP system time offset on sync data')
    plt.grid(True)

    # # TEST:
    # fig_5 = plt.figure(5)
    # ax_5 = plt.subplot(1, 1, 1)
    # ax_5.scatter(test_seq, test_delta)
    # plt.xlabel('data seq')
    # plt.ylabel('time difference (s)')
    # plt.title('recovery BT sync time - original sync time')

    plt.show()

if __name__ == "__main__":
   main()