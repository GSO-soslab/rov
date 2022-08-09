# !/usr/bin/env python

import csv
import pandas as pd

df = pd.read_csv(
  '/home/lin/Desktop/2022-03-30-usbl-01_usbl.csv', sep=",", header=None, 
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
         "38_CRP_Lat", "39_CRP_Lon", "40_CRP_Altitude", 
         "41_Mission_Plan",
         "42_Pressure", "43_Voltage", "44_Pressure_Offset", "45_Latitude", 
         "46_Unkown_1", "47_Unkown_2", "48_Unkown_3", "49_Unkown_4"])
        
name     =  list(df["1_Name"])
accuracy =  list(df["10_Accuracy"])
X =  list(df["11_Raw_X"])
Yaw =  list(df["32_Yaw"])



for i in range(len(name)):
  print(name[i])
print("")

for i in range(len(accuracy)):
  print(accuracy[i])      
print("")

for i in range(len(Yaw)):
  print(Yaw[i])
print("")
