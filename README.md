

- git clone https://github.com/GSO-soslab/rov 
- cd rov
- git submodule update --init --recursive

- rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y