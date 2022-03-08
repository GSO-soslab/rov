## ROV Operation Sheet

### Beach Pond lists:


**Calibration:**
```sh
# collect mag data

# do calibration
roslaunch calibrate_imu calibrator.launch
```


**Jetson Side:**
# set time
sudo timedatectl set-timezone EST
sudo timedatectl set-timezone America/Anchorage 
sudo date --set="2022-03-07 15:25:30.990"
sudo timedatectl set-timezone UTC

# Start mission
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
# Record data
```sh
rosbag record \
/rov/sensors/ahrs/imu/data /rov/sensors/ahrs/mag \
/rov/sensors/dvl/df21/df21_sync /rov/sensors/dvl/df3_sync /rov/sensors/dvl/dvl/dvl_sync /rov/sensors/dvl/pointcloud/ pointcloud_sync /rov/sensors/dvl/velocity/velocity_sync /rov/sensors/dvl/depth/depth_sync \
/rov/sensors/sonar/ping /rov/sensors/sonar/raw_img \
/rov/sensors/stereo/left/image_numbered/image_raw_sync /rov/sensors/stereo/right/image_numbered/image_raw_sync
```

**Topside:**
```sh
# start monitoring
roslaunch rov_remote visualization.launch
```
```sh
# start USBL server
sudo /etc/init.d/sinaps start
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


