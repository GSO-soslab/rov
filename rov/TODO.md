# ROV

## TODO

### 1) USBL:
- analysis data
- USBL power cable 

### 2) algorithm:
- KLT tracking *-THIS-*

### 3) underice dataset
- read paper  *-THIS-*
- applications:
    - ?

-------------------------------------------------

## Upgrade:


### 1) Arduino:

#### + PPS
- send NMEA string take too much time
    - remove unnecessary NMEA string
    - buffer for camera message
    - increase baud rate:
        - [related 1](https://forums.developer.nvidia.com/t/non-standard-baud-rate-for-the-uart/110323)
        - [related 2](https://forums.developer.nvidia.com/t/how-to-make-nanos-uart-work-at-about-8mbps-baudrate/110229#5418557)
        - [related 3](https://forums.developer.nvidia.com/t/ths0-uart-is-not-receiving-any-data/197054/12)

#### + Configuration:
- change good namespace
- change good init output
- synchronize_manager info/warn/error
    - system
    - science system
    - pps
    - cam

### 2) DVL Serial ROS driver:
- add pressure in the DVL message





-------------------------------------------------

