# !/usr/bin/env python

import csv
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

import rosbag
import rospy
from nav_msgs.msg import Odometry

### usage: python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/06_parse_usbl.py

###################################################################################################
####                              Separate data into ahrs and usbl                             ####
###################################################################################################

#### Create separated data filename

# data_filename = '/home/lin/develop/data/underIce/alaska/03_23/2022-03-24-data-01.csv'
data_filename = '/home/lin/develop/data/underIce/alaska/03_29/2022-03-29-data-01.csv'
# data_filename = '/home/lin/develop/data/underIce/alaska/03_30/2022-03-30-data-01.csv'

# generate new filename for separated data
index = data_filename.find('.csv')
ahrs_filename = data_filename[:index] + '_ahrs' + data_filename[index:] 
usbl_filename = data_filename[:index] + '_usbl' + data_filename[index:] 

print(' ')
print('saved ahrs file: %s' % ahrs_filename)
print('saved usbl file: %s' % usbl_filename)

#### Write separated data
file_ahrs = open(ahrs_filename, 'w')
file_usbl = open(usbl_filename, 'w')

with open(data_filename, mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        if row[0] == 'ahrs':
            writer = csv.writer(file_ahrs)
            writer.writerow(row)
            # print(row[0])

        if row[0] == 'usbl':
            writer = csv.writer(file_usbl)
            writer.writerow(row)
            # print(row[0])

file_ahrs.close()   
file_usbl.close()

print(' ')
print('finsih separate ahrs and usbl !!!')

###################################################################################################
####                                        Parse usbl data                                    ####
###################################################################################################


#### Get entire USBL data
# 2_Computer_Timestamp: 
#   - Navigation computer’s system time, millisecond
#   - after SiNAPS processed and save to database
#   - 1648581785941
# 3_Measurement_Timestamp: 
#   - Positioning timestamp
#   - example:1648581785.646
# 5_Transceiver_Timestamp:
#   - USBL time outputted the positioning data to SiNAPS
#   - the number of seconds elapsed since the device has been powered on.
#   - example: 64.510671
# 8_Positioning_Timestamp: 
#   - USBL time when received transponder’s reply
#   - example: not display in this section

df = pd.read_csv(
  usbl_filename, sep=",", header=None, 
  names=["1_Name", "2_Computer_Timestamp", "3_Measurement_Timestamp", 
         "4_Transponder_Address", "5_Transceiver_Timestamp", 
         "6_RSSI", "7_INTEGRITY", 
         "8_Positioning_Timestamp", "9_Propagation_Time", "10_Accuracy", 
         "11_Raw_X", "12_Raw_Y", "13_Raw_Z",
         "14_Raw_E", "15_Raw_N", "16_Raw_U", 
         "17_X_CRP", "18_Y_CRP", "19_Z_CRP", 
         "20_N_CRP", "21_E_CRP", "22_D_CRP", 
         "23_Transponder_Lat", "24_Transponder_Lon", "25_Transponder_Altitude", "26_GPS_Fix",
         "27_Raw_Roll", "28_Raw_Pitch", "29_Raw_Yaw", "30_Roll", "31_Pitch", "32_Yaw", 
         "33_GPS_Lat", "34_GPS_Lon", "35_GPS_Altitude", "36_GPS_Sentence", "37_GPS_Fix", 
         "38_CRP_Lat", "39_CRP_Lon", "40_CRP_Altitude", "41_Mission_Plan",
         "42_Pressure", "43_Voltage", "44_Pressure_Offset", "45_Latitude", 
         "46_Unkown_1", "47_Unkown_2", "48_Unkown_3", "49_Unkown_4"])

#### get each value
name                  =  list(df["1_Name"])
computer_time         =  list(df["2_Computer_Timestamp"])
measurement_time      =  list(df["3_Measurement_Timestamp"])
accuracy              =  list(df["10_Accuracy"]) # positioning accuracy
rssi                  =  list(df["6_RSSI"]) # Higher value represent strong signal
integrity             =  list(df["7_INTEGRITY"]) # Higher value means less distorted signal, less then 100 means bad
raw_X                 =  list(df["11_Raw_X"])
raw_Y                 =  list(df["12_Raw_Y"])
raw_Z                 =  list(df["13_Raw_Z"])
# raw_E =  list(df["14_Raw_E"])
# raw_N =  list(df["15_Raw_N"])
# raw_U =  list(df["16_Raw_U"])
# X_CRP =  list(df["17_X_CRP"])
# Y_CRP =  list(df["18_Y_CRP"])
# Z_CRP =  list(df["19_Z_CRP"])
# N_CRP =  list(df["20_N_CRP"])
# E_CRP =  list(df["21_E_CRP"])
# D_CRP =  list(df["22_D_CRP"])         

# check data size
if len(accuracy) != len(raw_X) != len(raw_Y) != len(raw_Z) != len(rssi) != len(integrity) != len(computer_time) != len(measurement_time):
    print('data size not right !')

# generate time step
time_step = [] 
for i in range(len(computer_time)):
    time_step.append(i)

print(' ')
print('finsih parse ahrs and usbl !!!')

###################################################################################################
####                                       Plot usbl data                                      ####
###################################################################################################

###########################         plot timeoffset        ########################### 

### generate time offset: 2_Computer_Timestamp - 8_Positioning_Timestamp

offset = []
for i in range(len(computer_time)):
    offset.append(computer_time[i]/1e3 - measurement_time[i])

### plot
fig = plt.figure(1)
ax = fig.add_subplot(111)
plot= ax.scatter(time_step, offset)
ax.locator_params(nbins=6)
ax.set_xlabel('timestep [scale]', fontsize=11)
ax.set_ylabel('time [s]', fontsize=11)
plt.tight_layout()
plt.title("timeoffset: 2_Computer_Timestamp - 8_Positioning_Timestamp")


###########################         plot XYZ        ########################### 

### Create cubic bounding box to simulate equal aspect ratio
### from: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to

max_range = np.array([max(raw_X)-min(raw_X), max(raw_Y)-min(raw_Y), max(raw_Z)-min(raw_Z)]).max() /2.0
mid_x = (max(raw_X) + min(raw_X)) * 0.5
mid_y = (max(raw_Y) + min(raw_Y)) * 0.5
mid_z = (max(raw_Z) + min(raw_Z)) * 0.5

# plot
fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')
traj= ax.scatter(raw_X, raw_Y, raw_Z,)
ax.locator_params(nbins=6)
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_zlabel('Z [m]', fontsize=11)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
plt.tight_layout()
plt.title("USBL RAW XYZ plot")


########################### filter ########################### 

threshold_accuracy = 3
threshold_rssi = -50
threshold_integrity = 100
threshold_time_min = 1648581841.23
threshold_time_max = 1648583440.19

filter_accuracy = []
filter_rssi = []
filter_intergity = []
filter_time = []

for i in range(len(measurement_time)):
    ### filter with accuracy
    if accuracy[i] < threshold_accuracy:
        filter_accuracy.append(1)
    else:
        filter_accuracy.append(0)

    ### filter with rssi
    if rssi[i] > threshold_rssi:
        filter_rssi.append(1)
    else:
        filter_rssi.append(0)

    ### filter with integrity
    if integrity[i] > threshold_integrity:
        filter_intergity.append(1)
    else:
        filter_intergity.append(0)

    ### filter with selected time duration
    if measurement_time[i] >= threshold_time_min and measurement_time[i] <= threshold_time_max:
        filter_time.append(1)
    else:
        filter_time.append(0)


########################### plot ########################### 
filtered_x = []
filtered_y = []
filtered_z = []

rules_accuracy = []
rules_rssi = []
rules_intergity = []
rules_time = []

for i in range(len(measurement_time)):
    if (filter_accuracy[i]  == 1 and 
        filter_rssi[i]      == 1 and 
        filter_intergity[i] == 1 and 
        filter_time[i]      == 1 ):

        ### get filtered position
        filtered_x.append(raw_X[i])
        filtered_y.append(raw_Y[i])
        filtered_z.append(raw_Z[i])

        ### get filtered rules for visualization
        rules_accuracy.append(accuracy[i])
        rules_rssi.append(rssi[i])
        rules_intergity.append(integrity[i])
        rules_time.append(i)

### plot with accuracy 
fig = plt.figure(3)
ax = fig.add_subplot(111, projection='3d')
traj= ax.scatter(filtered_x, filtered_y, filtered_z, c=rules_accuracy, s=5, cmap='jet')
height_bar = fig.colorbar(traj, pad=0.2)
height_bar.set_label("Accuracy [m]", fontsize=12)
ax.locator_params(nbins=6)
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_zlabel('Z [m]', fontsize=11)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
plt.tight_layout()
plt.title("USBL RAW XYZ plot with Accuracy filtered")


### plot with rssi 
fig = plt.figure(4)
ax = fig.add_subplot(111, projection='3d')
traj= ax.scatter(filtered_x, filtered_y, filtered_z, c=rules_rssi, s=5, cmap='jet')
height_bar = fig.colorbar(traj, pad=0.2)
height_bar.set_label("RSSI [dB re 1V]", fontsize=12)
ax.locator_params(nbins=6)
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_zlabel('Z [m]', fontsize=11)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
plt.tight_layout()
plt.title("USBL RAW XYZ plot with RSSI filtered")

### plot with intergity 
fig = plt.figure(5)
ax = fig.add_subplot(111, projection='3d')
traj= ax.scatter(filtered_x, filtered_y, filtered_z, c=rules_intergity, s=5, cmap='jet')
height_bar = fig.colorbar(traj, pad=0.2)
height_bar.set_label("INTEGRAITY [integer]", fontsize=12)
ax.locator_params(nbins=6)
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_zlabel('Z [m]', fontsize=11)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
plt.tight_layout()
plt.title("USBL RAW XYZ plot with INTEGRAITY filtered")

### plot with selected time 
fig = plt.figure(6)
ax = fig.add_subplot(111, projection='3d')
traj= ax.scatter(filtered_x, filtered_y, filtered_z, c=rules_time, s=5, cmap='jet')
height_bar = fig.colorbar(traj, pad=0.2)
height_bar.set_label("timestep [scalar]", fontsize=12)
ax.locator_params(nbins=6)
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_zlabel('Z [m]', fontsize=11)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
plt.tight_layout()
plt.title("USBL RAW XYZ plot with selected time duration filtered")

plt.show()


# ########################### plot XYZ with accuracy ########################### 

# # filtering
# filter_x = []
# filter_y = []
# filter_z = []
# filter_rule = []
# accuracy_max = max(accuracy)
# accuracy_threshold = 2
# filter_accuracy = []

# for i in range(len(raw_X)):
#     if accuracy[i] < accuracy_threshold:
#         filter_rule.append(accuracy[i])
#         filter_x.append(raw_X[i])
#         filter_y.append(raw_Y[i])
#         filter_z.append(raw_Z[i])

#         filter_accuracy.append(1)
#     else:
#         filter_accuracy.append(0)

# fig = plt.figure(3)
# ax = fig.add_subplot(111, projection='3d')
# traj= ax.scatter(filter_x, filter_y, filter_z, c=filter_rule, s=5, cmap='jet')
# height_bar = fig.colorbar(traj, pad=0.2)
# height_bar.set_label("Accuracy [m]", fontsize=12)
# ax.locator_params(nbins=6)
# ax.set_xlabel('X [m]', fontsize=11)
# ax.set_ylabel('Y [m]', fontsize=11)
# ax.set_zlabel('Z [m]', fontsize=11)
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)
# plt.tight_layout()
# plt.title("USBL RAW XYZ plot with Accuracy")


# ###########################    plot XYZ with RSSI    ###########################

# # filtering
# filter_x = []
# filter_y = []
# filter_z = []
# filter_rule = []
# rssi_min = min(rssi)
# rssi_threshold = -50
# filter_rssi = []

# for i in range(len(raw_X)):
#     if rssi[i] > rssi_threshold:
#         filter_rule.append(rssi[i])
#         filter_x.append(raw_X[i])
#         filter_y.append(raw_Y[i])
#         filter_z.append(raw_Z[i])

#         filter_rssi.append(1)
#     else:
#         filter_rssi.append(0)

# # plot
# fig = plt.figure(4)
# ax = fig.add_subplot(111, projection='3d')
# traj = ax.scatter(filter_x, filter_y, filter_z, c=filter_rule, s=5, cmap='jet')
# height_bar = fig.colorbar(traj, pad=0.2)
# height_bar.set_label("RSSI [dB re 1V]", fontsize=12)
# ax.locator_params(nbins=6)
# ax.set_xlabel('X [m]', fontsize=11)
# ax.set_ylabel('Y [m]', fontsize=11)
# ax.set_zlabel('Z [m]', fontsize=11)
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)
# plt.tight_layout()
# plt.title("USBL RAW XYZ plot with RSSI")

# ###########################          plot XYZ with integrity    ###########################
# # filtering
# filter_x = []
# filter_y = []
# filter_z = []
# filter_rule = []
# integrity_min = min(integrity)
# integrity_threshold = 100
# filter_intergity = []

# for i in range(len(raw_X)):
#     if integrity[i] > integrity_threshold:
#         filter_rule.append(integrity[i])
#         filter_x.append(raw_X[i])
#         filter_y.append(raw_Y[i])
#         filter_z.append(raw_Z[i])

#         filter_intergity.append(1)
#     else:
#         filter_intergity.append(0)

# # plot
# fig = plt.figure(5)
# ax = fig.add_subplot(111, projection='3d')
# traj = ax.scatter(filter_x, filter_y, filter_z, c=filter_rule, s=5, cmap='jet')
# height_bar = fig.colorbar(traj, pad=0.2)
# height_bar.set_label("INTEGRAITY [integer]", fontsize=12)
# ax.locator_params(nbins=6)
# ax.set_xlabel('X [m]', fontsize=11)
# ax.set_ylabel('Y [m]', fontsize=11)
# ax.set_zlabel('Z [m]', fontsize=11)
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)
# plt.tight_layout()
# plt.title("USBL RAW XYZ plot with INTEGRAITY")

# ###########################           plot XYZ with time      ###########################
# time_max = max(measurement_time)
# time_min = min(measurement_time)
# filter_x = []
# filter_y = []
# filter_z = []
# filter_rule = []
# filter_time = []
# time_threshold_min = 1648581841.23
# time_threshold_max = 1648583440.19

# for i in range(len(measurement_time)):
#     if measurement_time[i] >= time_threshold_min and measurement_time[i] <= time_threshold_max:
#         filter_x.append(raw_X[i])
#         filter_y.append(raw_Y[i])
#         filter_z.append(raw_Z[i])
#         filter_rule.append(i)

#         filter_time.append(1)
#     else:
#         filter_time.append(0)

# fig = plt.figure(6)
# ax = fig.add_subplot(111, projection='3d')
# traj = ax.scatter(filter_x, filter_y, filter_z, c=filter_rule, s=5, cmap='jet')
# height_bar = fig.colorbar(traj, pad=0.2)
# height_bar.set_label("time_step [scalar]", fontsize=12)
# ax.locator_params(nbins=6)
# ax.set_xlabel('X [m]', fontsize=11)
# ax.set_ylabel('Y [m]', fontsize=11)
# ax.set_zlabel('Z [m]', fontsize=11)
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)
# plt.tight_layout()
# plt.title("USBL RAW XYZ plot with time step")


# ###########################           save to rosbag      ###########################

# # save
# outbag = rosbag.Bag("/home/lin/develop/data/underIce/alaska/03_29/usbl_test.bag", 'w')
# print("working on the parsing ......")
# print()

# if len(filter_accuracy) != len(filter_rssi) != len(filter_intergity) != len(measurement_time) != len(filter_time):
#     print('filtered data size not right !')

# overall_x = []
# overall_y = []
# overall_z = []
# for i in range(len(measurement_time)):

#     if filter_accuracy[i]==1 and filter_rssi[i]==1 and filter_intergity[i]==1 and filter_time[i]==1:
#         msg = Odometry()
#         msg.header.frame_id = "usbl"
#         msg.header.stamp = rospy.Time.from_sec(measurement_time[i])
#         msg.child_frame_id = "modem"
#         msg.pose.pose.position.x = raw_X[i]
#         msg.pose.pose.position.y = raw_Y[i]
#         msg.pose.pose.position.z = raw_Z[i]
#         outbag.write("/usbl_odom", msg, msg.header.stamp)

#         overall_x.append(raw_X[i])
#         overall_y.append(raw_Y[i])
#         overall_z.append(raw_Z[i])

# print("finsih save rosbag!")
# outbag.close()

# # plot overall filtered data
# fig = plt.figure(7)
# ax = fig.add_subplot(111, projection='3d')
# traj = ax.scatter(overall_x, overall_y, overall_z)
# ax.locator_params(nbins=6)
# ax.set_xlabel('X [m]', fontsize=11)
# ax.set_ylabel('Y [m]', fontsize=11)
# ax.set_zlabel('Z [m]', fontsize=11)
# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)
# plt.tight_layout()
# plt.title("USBL RAW XYZ plot with overall filtering!")

