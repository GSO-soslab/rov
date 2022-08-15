# !/usr/bin/env python

import csv
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
name      =  list(df["1_Name"])
t         =  list(df["2_Computer_Timestamp"])
accuracy  =  list(df["10_Accuracy"])
rssi      =  list(df["6_RSSI"])
integrity =  list(df["7_INTEGRITY"])
raw_X     =  list(df["11_Raw_X"])
raw_Y     =  list(df["12_Raw_Y"])
raw_Z     =  list(df["13_Raw_Z"])
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
if len(accuracy) != len(raw_X) != len(raw_Y) != len(raw_Z) != len(rssi) != len(integrity) != len(t):
    print('data size not right !')

# generate time step
time_step = [] 
for i in range(len(t)):
    time_step.append(i)

print(' ')
print('finsih parse ahrs and usbl !!!')

###################################################################################################
####                                       Plot usbl data                                      ####
###################################################################################################

### plot accuracy:
# fig_1 = plt.figure(1)
# plt.plot(x_axis, accuracy)
# plt.tight_layout()

### Create cubic bounding box to simulate equal aspect ratio
### from: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to

max_range = np.array([max(raw_X)-min(raw_X), max(raw_Y)-min(raw_Y), max(raw_Z)-min(raw_Z)]).max() /2.0
mid_x = (max(raw_X) + min(raw_X)) * 0.5
mid_y = (max(raw_Y) + min(raw_Y)) * 0.5
mid_z = (max(raw_Z) + min(raw_Z)) * 0.5

###########################         plot XYZ        ########################### 
# plot
fig = plt.figure(1)
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

########################### plot XYZ with accuracy ########################### 

# filtering
fliter_x = []
fliter_y = []
fliter_z = []
fliter_rule = []
for i in range(len(raw_X)):
    if accuracy[i] < 10:
        fliter_rule.append(accuracy[i])
        fliter_x.append(raw_X[i])
        fliter_y.append(raw_Y[i])
        fliter_z.append(raw_Z[i])
# plot
fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')
traj= ax.scatter(fliter_x, fliter_y, fliter_z, c=fliter_rule, s=5, cmap='jet')
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
plt.title("USBL RAW XYZ plot with Accuracy")

###########################    plot XYZ with RSSI    ###########################

# filtering
fliter_x = []
fliter_y = []
fliter_z = []
fliter_rule = []
for i in range(len(raw_X)):
    if rssi[i] > -100:
        fliter_rule.append(rssi[i])
        fliter_x.append(raw_X[i])
        fliter_y.append(raw_Y[i])
        fliter_z.append(raw_Z[i])
# plot
fig = plt.figure(3)
ax = fig.add_subplot(111, projection='3d')
traj = ax.scatter(fliter_x, fliter_y, fliter_z, c=fliter_rule, s=5, cmap='jet')
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
plt.title("USBL RAW XYZ plot with RSSI")

###########################          plot XYZ with integrity    ###########################
# filtering
fliter_x = []
fliter_y = []
fliter_z = []
fliter_rule = []
for i in range(len(raw_X)):
    if integrity[i] < 500:
        fliter_rule.append(integrity[i])
        fliter_x.append(raw_X[i])
        fliter_y.append(raw_Y[i])
        fliter_z.append(raw_Z[i])
# plot
fig = plt.figure(4)
ax = fig.add_subplot(111, projection='3d')
traj = ax.scatter(fliter_x, fliter_y, fliter_z, c=fliter_rule, s=5, cmap='jet')
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
plt.title("USBL RAW XYZ plot with INTEGRAITY")

###########################           plot XYZ with time      ###########################
fig = plt.figure(5)
ax = fig.add_subplot(111, projection='3d')
traj = ax.scatter(raw_X, raw_Y, raw_Z, c=time_step, s=5, cmap='jet')
height_bar = fig.colorbar(traj, pad=0.2)
height_bar.set_label("time_step [scalar]", fontsize=12)

ax.locator_params(nbins=6)
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_zlabel('Z [m]', fontsize=11)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
plt.tight_layout()
plt.title("USBL RAW XYZ plot with time step")

plt.show()