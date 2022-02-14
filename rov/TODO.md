# ROV

## Hardware:

-------------------------------------------------

## Software drivers:

### PPS
- send NMEA string take too much time
    - remove unnecessary NMEA string
    - buffer for camera message
    - increase baud rate:
        - [related 1](https://forums.developer.nvidia.com/t/non-standard-baud-rate-for-the-uart/110323)
        - [related 2](https://forums.developer.nvidia.com/t/how-to-make-nanos-uart-work-at-about-8mbps-baudrate/110229#5418557)
        - [related 3](https://forums.developer.nvidia.com/t/ths0-uart-is-not-receiving-any-data/197054/12)

### DVL:
- add pressure in the DVL message

### Science system:
- make it as class object in arduino

### Battery measurement
- make it as class object in arduino

### Configuration:
- change good namespace
- change good init output
- synchronize_manager info/warn/error
    - system
    - science system
    - pps
    - cam


-------------------------------------------------

## Time synchronization (**IMPORTANT**)

### LED 
- flash with camera 

### GPS time
- grab gps time message 

### Cameras:
- change Jetson system time clock will lead to : `Time candidates buffer overflow`

### DVL NTP
- setup NTP server as Jetson 

### Jetson
- use UTC time, not local time?


-------------------------------------------------

## Algorithms:

### robot_localization
- online test with neternet DVL
- update odom in the global frame for DVL_serial(tf publish global with odom)
- keep serial/ethernet to the same

### mag calibration:
- keep python2 and python3 similar to same, upload to github

### KLT feature tracking


### msckf_dvio