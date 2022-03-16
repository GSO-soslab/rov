#!/usr/bin/python

import os
import sys


# dir_name = '/home/soslab/Projects/under_ice_navigation/dataset/underice/alaska/calibration/left_raw/'

if len(sys.argv) < 2:
    print("usage: python print_filename.py path")
    exit()

## check path
dir_name = sys.argv[1]
if dir_name[-1] != '/':
    dir_name = dir_name + '/'

num = 0

# Get list of all files in a given directory sorted by name
list_of_files = sorted( filter( lambda x: os.path.isfile(os.path.join(dir_name, x)),
                        os.listdir(dir_name) ) )

for file_name in list_of_files:
    num += 1
    print(dir_name+file_name)

print('Finished with num=', num)