# !/usr/bin/env python

import csv
import pandas as pd

###################################################################################################
####                              Separate data into ahrs and usbl                             ####
###################################################################################################

#### Create separated data filename
# data_filename = '/home/lin/Desktop/2022-03-30-usbl-01.csv'
data_filename = '/home/lin/develop/data/underIce/alaska/03_30/2022-03-30-usbl-01.csv'
index = data_filename.find('.csv')
# generate new filename for separated data
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
name     =  list(df["1_Name"])
t        =  list(df["2_Computer_Timestamp"])
accuracy =  list(df["10_Accuracy"])
raw_X =  list(df["11_Raw_X"])
raw_Y =  list(df["12_Raw_Y"])
raw_Z =  list(df["13_Raw_Z"])
raw_E =  list(df["14_Raw_E"])
raw_N =  list(df["15_Raw_N"])
raw_U =  list(df["16_Raw_U"])
X_CRP =  list(df["17_X_CRP"])
Y_CRP =  list(df["18_Y_CRP"])
Z_CRP =  list(df["19_Z_CRP"])
N_CRP =  list(df["20_N_CRP"])
E_CRP =  list(df["21_E_CRP"])
D_CRP =  list(df["22_D_CRP"])         

# count = 0
# for i in range(len(name)):
#   print(name[i])
#   count +=1
# print("")
# print('name total: %s' % count)

###################################################################################################
####                                       Plot usbl data                                      ####
###################################################################################################
