# ROV Operation Sheet

## Mission Plan

### Launch Mission

#### Trim ROV

#### Time setup for system
- save Topside GPS date: `python /home/soslab/Projects/under_ice_navigation/rov_ws/src/rov/rov_remote/tools/scripts/save_gps.py`
- screenshot when power-on the ROV
- ssh to Jetson: `ssh soslab@192.168.2.3`
<!-- - monitor system time for science system in Field Laptop:`watch -n -0.1 date +%s` -->
- set jetson time to UTC:  `sudo date --set="2022-03-30 18:55:40.000"`, copy from topside


#### Sensor setting brefore mission:
- record mag calibration data sometime
    ```sh
    rosbag record \
    /rov/sensors/ahrs/mag
    ```
- put ROV in the ice opening
- record pressure sensor from DVL:`roslaunch rov_onboard sensors.launch`
    ```sh
    rosbag record \
    /rov/sensors/dvl/df21
    ```
- adjust camera exposure during LED is ON
- adjust SONAR parameters

#### Start Sensor system:
- launch onboard synchronizer: `roslaunch rov_onboard synchronizer.launch` 
- launch topside visualizer: `roslaunch rov_remote visualization.launch`
- set arduino clock: click `Computer_Clock` in RQT
- set Jetson NTP time sync: `sudo sh ~/Desktop/run_time_sync.sh`
- GPSD time ahead:
    - `3_28: 886 ms, 892ms `
    - `3_29: 883 ms`
    - `3_30: 908 ms`
- check Topside: `sudo systemctl restart chronyd & watch -n -0.1 chronyc sources -v`
- launch onboard sensors: `roslaunch rov_onboard sensors.launch`
<!-- - timer the Arduino -->
- verify time synchronization: 
    - AHRS: check if it's get GPS
    - DVL: check if it's initialized
    - Stereo: check if it's initialized

#### Start Mission:
- Drive ROV below ice 1.5 or 2 meter 
- check topside is time synchronized
- launch Topside USBL: `sudo /etc/init.d/sinaps start`
- create marker, then start USBL ping
- launch onboard odomtery: `roslaunch rov_processing alaska_nav.launch`
- record data: 
    ```sh
    # 1 hour will record 409 GB
    rosbag record --split --duration=5m \
    /rov/sensors/ahrs/imu/data /rov/sensors/ahrs/mag /rov/sensors/ahrs/imu/calib \
    /rov/sensors/dvl/df21/df21_sync /rov/sensors/dvl/df3_sync \
    /rov/sensors/dvl/df21 /rov/sensors/dvl/df3 \
    /rov/sensors/sonar/ping \
    /rov/sensors/stereo/left/image_numbered/image_raw_sync /rov/sensors/stereo/right/image_numbered/image_raw_sync /rov/sensors/stereo/right/calib
    ```
- timer the rosbag

- record test
    ```sh
    rosbag record \
    /rov/sensors/ahrs/imu/calib \
    /rov/sensors/ahrs/imu/data \
    /rov/sensors/ahrs/mag \
    /rov/sensors/stereo/left/image_numbered/image_raw_sync \
    /rov/sensors/stereo/left/image_numbered/image_raw_sync/compressed \
    /rov/sensors/stereo/right/calib \
    /rov/sensors/stereo/right/image_numbered/image_raw_sync \
    /rov/sensors/stereo/right/image_numbered/image_raw_sync/compressed
    ```

---------------------------------------------------------------------------------------------------
