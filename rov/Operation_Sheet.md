# ROV Operation Sheet

## Mission Plan

### Launch Mission

#### Trim ROV

#### Time setup for system
- monitor system time for science system in Field Laptop:`watch -n -0.1 date +%s`
- cancel system time and power-on the ROV at same time, record time in the log book
- ssh to Jetson: `ssh soslab@192.168.2.3`
- set jetson time to UTC:  `sudo date --set="2022-03-25 06:41:30.990"`, copy from topside
- GPSD time ahead:
    - `?`
    - `?`

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
- launch onboard sensors: `roslaunch rov_onboard sensors.launch`
- verify time synchronization: 
    - Jetson: `watch -n -0.1 chronyc sources -v`
    - Topside: `watch -n -0.1 chronyc sources -v`
    - AHRS: check if it's get GPS
    - DVL: check if it's initialized
    - Stereo: check if it's initialized

#### Start Mission:
- Drive ROV below ice 1.5 or 2 meter 
- launch Topside USBL: `sudo /etc/init.d/sinaps start`
- save Topside GPS date: `python /home/soslab/Projects/under_ice_navigation/rov_ws/src/rov/rov_remote/tools/scripts/save_gps.py`
- launch onboard odomtery: `roslaunch rov_processing alaska_nav.launch`
- record data: 
    ```sh
    # 1 hour will record 409 GB
    rosbag record  --split --duration=5m \
    /rov/sensors/ahrs/imu/data /rov/sensors/ahrs/mag /rov/sensors/ahrs/imu/calib \
    /rov/sensors/dvl/df21/df21_sync /rov/sensors/dvl/df3_sync \
    /rov/sensors/sonar/ping \
    /rov/sensors/stereo/left/image_numbered/image_raw_sync /rov/sensors/stereo/right/image_numbered/image_raw_sync /rov/sensors/stereo/right/calib
    ```
- timer the rosbag

- record camera images without light on
    ```sh
    rosbag record \
    /rov/sensors/stereo/left/image_numbered/image_raw_sync /rov/sensors/stereo/right/image_numbered/image_raw_sync
    ```

---------------------------------------------------------------------------------------------------
