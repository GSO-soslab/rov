# ROV

## TODO

### 1) USBL:
- analysis data
- USBL power cable 

### 2) algorithm:
- KLT tracking *-THIS-*

### 3) underice dataset

- Sensors:
    - Nortek DVL 1000
        - bottom track: 7hz
        - current profile: 1hz
    - Microstrain AHRS:
        - Acce/Gyro/Mag: 200hz
    - FLIR Stereo-cam:
        - images_1: 10hz, 1616x1200
        - images_2: 10hz, 1616x1200
    - Blueprint Sonar 750D:
        - image: 10hz
    - Eveologics USBL:
        - pose: ~1hz

- Data Format:
    - time:
        - DVL IO timestamp
        - AHRS IO timestamp
        - Stereo-cam IO timestamp
        - Sonar IO timestamp
    - experiments:
            - cloudy: image without LED; half, full LED
            - sunny:  no, half, full LED ?
- Calibrations:
    - Camera:
        - Geometric
        - Photometric(response, attenuation, and exposure times): [A Photometrically Calibrated Benchmark For Monocular Visual Odometry](https://arxiv.org/abs/1607.02555)
            - exposure time
            - response
            - vignette
        - extrinsic: IMU-Cam
    - DVL:
        - extrinsic: IMU-DVL
    - Sonar:
        - extrinsic: IMU-Sonar
        - timeoffset: IMU-Sonar
    - LED:
        - Light model/source: [A Benchmark for Visual-Inertial Odometry Systems Employing Onboard Illumination](https://ieeexplore.ieee.org/document/8968554)


- Evaluation Metrics:
    - ground truth with our method: fuse USBL + DVL + IMU + stereo-cam (loop_closure) + sonar (loop_closure)
    - RPE, ATE

- Benchmarks:
    - direct-method:
        - VO: DSO, SVO ...
        - VIO: ROVIO ...
        - Stereo-Mono: stereo-DSO...
    - KLT-method:
    - SFM:

- Applications:
    - VIO
    - multi-sensors SLAM
    - online calibration (timeoffset, Photometric...)
    - sonar related

-------------------------------------------------

