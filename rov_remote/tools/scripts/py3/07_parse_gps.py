#!/usr/bin/python

#usage: python3 /home/lin/develop/ros/rov_ws/src/rov/rov_remote/tools/scripts/py3/07_parse_gps.py

import csv
import matplotlib.pyplot as plt

time_pc = []
time_gps = []

# csv_file = '/home/lin/develop/data/underIce/alaska/03_30/1648665819.csv'
# csv_file = '/home/lin/develop/data/underIce/alaska/03_29/1648578490.csv'
csv_file = '/home/lin/develop/data/underIce/alaska/03_29/1648580759.csv'

with open(csv_file, mode='r') as csv_file:
    csv_reader = csv.DictReader(csv_file)
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            print(f'Column names are {", ".join(row)}')
            line_count += 1

        time_pc.append(float(row["computer_time"]))
        time_gps.append(float(row["GPS_time"]))
        line_count += 1

        # if line_count == 5:
        # 	break

    # print(time_pc)
    # print(time_gps)
    print(f'Processed {line_count} lines.')

# prepare computer time timeoffset for each 2 stamp
delta_pc = [] 
num = []
for i in range(1, len(time_pc)):
    delta_pc.append(time_pc[i] - time_pc[i-1])
    num.append(i)

delta_pc_gps = []
num_2 = []
init_time = 0
array_time = [] 
for i in range(len(time_pc)):
    delta_pc_gps.append(time_pc[i] - time_gps[i])
    if i==0:
        array_time.append(init_time)
        init_time = time_gps[i]
    else:
        array_time.append(time_gps[i] - init_time)
    # num_2.append(i)

# plot
fig_1 = plt.figure(1)
ax_1 = plt.subplot(1, 1, 1)
ax_1.scatter(num, delta_pc)
plt.xlabel('data num')
plt.ylabel('computer time difference (s)')
plt.title('check topside time')   

#####################################

fig_2 = plt.figure(2)
ax_2 = plt.subplot(1, 1, 1)
ax_2.scatter(array_time, delta_pc_gps)
plt.xlabel('data num')
plt.ylabel('time difference between laptop and gps(s)')
plt.title('check topside time') 

fig = plt.figure(2)
ax = fig.add_subplot(111)
plot= ax.scatter(array_time, delta_pc_gps)
ax.locator_params(nbins=10)
ax.set_xlabel('time [s]', fontsize=20)
ax.set_ylabel('timeoffset [s]', fontsize=20)
plt.tight_layout()
plt.title("Topside: sync time - gps time",fontsize=20)

plt.grid(True)
plt.show()