# Calibration:

## Opencv Camera Calibration:

- Save images:`roslaunch rov_remote tools_saveImg.launch`
- Select good images
- use python script to print selected files(modify fold path):`python print_filename.py`
- Modify image numbers of `opencv_calib_10x7.xml` and image path of `images_list.xml`
- `cd /home/soslab/Projects/under_ice_navigation/rov_ws/devel/lib/rov_remote`
- `./tools_calibCam_node ../../../src/rov/rov_remote/config/calibration/opencv_calib_10x7.xml`


## Kalibr Stereo Calibration:


## kalibr Camera-IMU Calibration:


# Test:
```sh
$ roscd rov_remote/tools/scripts
$ python verify_timeoffset.py  \
  /home/soslab/Projects/under_ice_navigation/dataset/underice/alaska/calibration/dvl.bag \
  /rov/sensors/dvl/df21/df21_sync
```