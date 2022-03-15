# Calibration:

## Opencv Camera Calibration:

- Save images:`roslaunch rov_remote tools_saveImg.launch`
- Select good images
- use python script to print selected files(modify fold path):`python print_filename.py`
- Modify image numbers of `opencv_calib_10x7.xml` and image path of `images_list.xml`
- `./tools_calibCam_node ../../../src/rov/rov_remote/config/calibration/opencv_calib_10x7.xml`


## Kalibr Stereo Calibration:


## kalibr Camera-IMU Calibration: