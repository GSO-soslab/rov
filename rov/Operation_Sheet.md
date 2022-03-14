# ROV Operation Sheet

## Mission Plan

#### Before Mission
- record air pressure
- collect mag calibration dataset

### Launch Mission
- set jetson time to UTC:  `sudo date --set="2022-02-14 10:21:30.990"`
- launch onboard synchronizer: `roslaunch rov_onboard synchronizer.launch` 
- launch topside visualizer: `roslaunch rov_remote visualization.launch`
- set arduino clock: click `Computer_Clock` in RQT
- set Jetson NTP time sync: `sudo sh ~/Desktop/run_time_sync.sh`
- launch onboard sensors: `roslaunch rov_onboard sensors.launch`
- verify time synchronization: 
    - Jetson: `watch -n -0.1 chronyc sources -v`
    - Topside: `watch -n -0.1 chronyc sources -v`
    - AHRS: check if it's get GPS
    - DVL: check if it's initialized
    - Stereo: check if it's initialized
- launch onboard odomtery: `roslaunch rov_processing alaska_nav.launch`
- launch Topside USBL: `sudo /etc/init.d/sinaps start`
- record data: 
    ```sh
    rosbag record \
    /rov/sensors/ahrs/imu/data /rov/sensors/ahrs/mag /rov/sensors/ahrs/imu/calib \
    /rov/sensors/dvl/df21/df21_sync /rov/sensors/dvl/df3_sync \
    /rov/sensors/sonar/ping /rov/sensors/sonar/raw_img \
    /rov/sensors/stereo/left/image_numbered/image_raw_sync /rov/sensors/stereo/right/image_numbered/image_raw_sync /rov/sensors/stereo/right/calib
    ```
- record test data:
    ```sh
    rosbag record \
    /rov/sensors/ahrs/imu/calib \
    /rov/sensors/stereo/right/calib
    ```

---------------------------------------------------------------------------------------------------
