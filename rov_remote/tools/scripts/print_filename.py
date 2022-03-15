#!/usr/bin/python

import os

dir_name = '/home/soslab/Desktop/selected/'
change_base = "1"
num = 0
tmp = 0


# Get list of all files in a given directory sorted by name
list_of_files = sorted( filter( lambda x: os.path.isfile(os.path.join(dir_name, x)),
                        os.listdir(dir_name) ) )


for file_name in list_of_files:
    print(dir_name+file_name)

print('All finished !!')