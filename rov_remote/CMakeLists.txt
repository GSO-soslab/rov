cmake_minimum_required(VERSION 2.8.3)

project(rov_remote)

add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  tf
  sensor_msgs
  tf_conversions
  )

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS
    roscpp
    rospy
    nav_msgs
    geometry_msgs
    tf
    sensor_msgs
    tf_conversions
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# message(STATUS "    testestestestest: $${PROJECT_SOURCE_DIR}")


find_package(xacro REQUIRED)
xacro_add_files(${CMAKE_CURRENT_SOURCE_DIR}/urdf/rov.urdf.xacro TARGET rov_remote)
install(FILES
    urdf/rov.urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# test
add_executable(path_node test/path_pub.cpp)
target_link_libraries(path_node
  ${catkin_LIBRARIES}
)

add_executable(eigen_node test/eigen_to_tf.cpp)
target_link_libraries(eigen_node
  ${catkin_LIBRARIES}
)