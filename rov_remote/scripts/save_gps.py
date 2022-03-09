#! /usr/bin/python

#### refereces:
# http://www.danmandle.com/blog/getting-gpsd-to-work-with-python/
# https://gpsd.gitlab.io/gpsd/gpsd_json.html

import os
from gps import *
from time import *
import time
import threading
import numpy as np
import datetime

pathName = "/home/soslab/Projects/under_ice_navigation/rov_ws/src/rov/rov_remote/result/gps/"
fileName = None
gps_time_updated = 0 
gpsd = None #seting the global variable
 
os.system('clear') #clear the terminal (optional)
 
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

def logData(data):
  logFile = open(fileName,"a")
  logFile.write(data + ",")
  logFile.close()

def logDataEnd(data):
  logFile = open(fileName,"a")
  logFile.write(data + "\n")
  logFile.close()

if __name__ == '__main__':
  fileName = "{}{}{}".format(pathName, str(datetime.datetime.now().strftime('%s')), ".csv")
  logFile = open(fileName,"a")
  logFile.write("computer_time,GPS_time,latitude,longitude,altitude,track,mode\n")
  logFile.close()

  gpsp = GpsPoller() # create the thread
  try:
    gpsp.start() # start it up
    while True:
 
      # os.system('clear')
      
      #### convert unicode to unix epoch time
      gps_time = gpsd.fix.time
      gps_epoch = 0
      if isinstance(gps_time, float):
        print('waitting...')
      elif isinstance(gps_time, unicode):
        ascii_string = gps_time.encode('ascii', 'ignore')
        year   = int(ascii_string[0:4])
        month  = int(ascii_string[5:7])
        day    = int(ascii_string[8:10])
        hour   = int(ascii_string[11:13])
        minute = int(ascii_string[14:16])
        second = int(ascii_string[17:19])
        gps_epoch = datetime.datetime(year, month, day, hour, minute, second).strftime('%s')

      #### check gps time is updated
      if gps_epoch > gps_time_updated :

        ## log computer time
        sys_time = time.time()
        logData('{0:.6f}'.format(sys_time))
        ## log GPS time
        gps_time_updated = gps_epoch
        logData(str(gps_time_updated))
        ## log latitude,
        logData(str(gpsd.fix.latitude))
        ## log longitude,
        logData(str(gpsd.fix.longitude))
        ## log altitude
        logData(str(gpsd.fix.altitude))
        ## log track
        logData(str(gpsd.fix.track))
        ## log mode
        logDataEnd(str(gpsd.fix.mode))

        print
        print('--------------Saved--------------------------')
        print('Computer time ', sys_time)
        print("GPS time: ", gps_time_updated)
 
      time.sleep(0.1) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("\nKilling Thread...")
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print("Done.\nExiting.")