# Jetson with Ubuntu 18.04

### Download
- git clone https://github.com/GSO-soslab/rov 
- cd rov
- git submodule update --init --recursive
- rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

### dependency:
**FLIR:**
- install spinnaker

**Arduino:**
- install Arduino:
  ```sh
  cd Downloads
  wget -N https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz
  tar -xvf arduino-1.8.19-linuxaarch64.tar.xz
  cd arduino-1.8.19
  sudo ./install.sh
  ```
- USB:
  ```sh
  sudo usermod -a -G dialout tty soslab
  sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ACTION==\"add\", ATTRS{idVendor}==\"2341\", ATTRS{idProduct}==\"804d\", SYMLINK+=\"arduino_zero\"" > /etc/udev/rules.d/99-usb-arduino.rules'
  sudo reboot
  ```
- install arduino lib: Ardino SAMD Board(32-bits ARM Cortex-M0+)

- Rebuild ros_lib:
  ```sh
  cd ~/Arduino/libraries/# generate ros-arduino-library
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
  # add Arduino Zero USB serial support for Ubuntu 18.04
  cp ~/path_to/rov/config/arduino/ArduinoHardware.h /path_to/ros_lib 
  # increase Arduino Zero buffer
  cp ~/path_to/rov/config/arduino/ros.h /path_to/ros_lib
  ```
- Compile:
  ```sh
  # check device
  ls -ln /dev/arduino_zero
  # upload
  cd ~/Develop/dev/arduino
  arduino --upload synchronizer_arduino/synchronizer_arduino.ino --port /dev/ttyACM1
  # verify 
  arduino --verify synchronizer_arduino/synchronizer_arduino.ino
  ```

**Microstrain:**
- [check here](/rov_devices/ROS_MSCL/README.md)

**oculus sonar**:
- [check here](/rov_devices/sonar_oculus/README.md)

**ROS:**
- [ros base](http://wiki.ros.org/melodic/Installation/Ubuntu)
- ros dependency:
  ```sh
    sudo apt-get install ros-melodic-image-transport 
    ros-melodic-rosserial ros-melodic-rosserial-arduino 
    ros-melodic-camera-info-manager ros-melodic-diagnostic-updater
    ros-melodic-roslint ros-melodic-serial ros-melodic-cv-bridge
    ros-melodic-tf2-geometry-msgs ros-melodic-image-proc
  ```

### Update the rov

- git submodule update --recursive --remote
- git pull