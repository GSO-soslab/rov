cmake_minimum_required(VERSION 3.0.2)
project(rov_processing)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)


### Find ROS package
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs rosbag sensor_msgs nav_msgs
  geometry_msgs message_generation image_transport
   tf_conversions eigen_conversions
  dynamic_reconfigure message_filters  
  ds_sensor_msgs nortek_dvl
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
# generate_dynamic_reconfigure_options(
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES feature_tracker
 CATKIN_DEPENDS roscpp rospy std_msgs rosbag sensor_msgs dynamic_reconfigure
                geometry_msgs message_runtime nortek_dvl tf_conversions eigen_conversions
                message_filters image_transport visualization_msgs tf nav_msgs ds_sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  /home/soslab/Develop/3rdparty/kalman-cpp
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${Boost_LIBRARY_DIR})

set(Kalman_LIBS /home/soslab/Develop/3rdparty/kalman-cpp/kalman.cpp )

##########################################
############### build lib ###############
##########################################


##########################################
############### build src ###############
##########################################
add_executable(proc_dvl_serial src/proc_dvl_serial.cpp ${Kalman_LIBS})
target_link_libraries(proc_dvl_serial ${catkin_LIBRARIES})
add_dependencies(proc_dvl_serial nortek_dvl_generate_messages_cpp)

add_executable(proc_dvl_ethernet src/proc_dvl_ethernet.cpp ${Kalman_LIBS})
target_link_libraries(proc_dvl_ethernet ${catkin_LIBRARIES} )
add_dependencies(proc_dvl_ethernet ds_sensor_msgs_generate_messages_cpp)

add_executable(pub_path src/pub_path.cpp )
target_link_libraries(pub_path ${catkin_LIBRARIES})

##########################################
############### build test ###############
##########################################



