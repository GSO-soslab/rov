

### Download
- git clone https://github.com/GSO-soslab/rov 
- cd rov
- git submodule update --init --recursive

- rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

### dependency:
**FLIR:**
- install spinnaker

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
    ros-melodic-tf2-geometry-msgs
  ```

### Update the rov

- git submodule update --recursive --remote
- git pull