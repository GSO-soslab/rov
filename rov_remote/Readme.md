## ROV Operation Sheet

### Beach Pond lists:


**Calibration:**
```sh
# collect mag data

# do calibration
roslaunch calibrate_imu calibrator.launch
```


**Jetson Side:**
```sh
# ssh to jetson
ssh soslab@192.168.2.3
# set time
sudo date --set="2022-02-14 10:21:30.990"
# start synchronizer 
roslaunch rov_onboard synchronizer.launch
# start all the senosrs
roslaunch rov_onboard sensors.launch
# after do time sync, start robot_localization
roslaunch rov_processing beachPond_nav.launch
```

**Topside:**
```sh
# start monitoring
roslaunch rov_remote visualization.launch
```





### Notes:

### Jetson

**Time setup**:
```sh
sudo date --set="2022-02-10 15:05:30.990"
```


### Pi

**Connection**:
```sh
ssh pi@192.168.2.2 # password: companion
```

### Arduino

**Compile**:
```sh
# check device
ls -ln /dev/arduino_zero
# upload
cd ~/Develop/dev/arduino
arduino --upload synchronizer_arduino/synchronizer_arduino.ino --port /dev/ttyACM1
# verify 
arduino --verify synchronizer_arduino/synchronizer_arduino.ino
```


