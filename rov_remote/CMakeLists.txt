cmake_minimum_required(VERSION 2.8.3)

project(rov_remote)

# find_package(OpenCV REQUIRED)
# message(STATUS "OpenCV library status:")
# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy nav_msgs geometry_msgs
  tf sensor_msgs tf_conversions 
  # cv_bridge
  )

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS
    roscpp rospy nav_msgs geometry_msgs
    tf sensor_msgs tf_conversions 
    # cv_bridge
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)

# message(STATUS "    testestestestest: $${PROJECT_SOURCE_DIR}")


find_package(xacro REQUIRED)
xacro_add_files(${CMAKE_CURRENT_SOURCE_DIR}/urdf/rov.urdf.xacro TARGET rov_remote)
install(FILES
    urdf/rov.urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# tools
# add_executable(tools_saveImg_node tools/cpp/save_images.cpp)
# target_link_libraries(tools_saveImg_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )

# add_executable( tools_calibCam_node tools/cpp/calibrate_camera.cpp )
# target_link_libraries( tools_calibCam_node ${OpenCV_LIBS} )

# test
add_executable(path_node test/path_pub.cpp)
target_link_libraries(path_node
  ${catkin_LIBRARIES}
)

add_executable(eigen_node test/eigen_to_tf.cpp)
target_link_libraries(eigen_node
  ${catkin_LIBRARIES}
)