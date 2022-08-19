cmake_minimum_required(VERSION 2.8.3)

project(rov_remote)

# The target architecture.
# For cross-compilation this should still work as you're supposed to set this var.
# X64 mean 64-bit generically, whether AMD64 or AARCH64.
message(STATUS ${CMAKE_SYSTEM_PROCESSOR})

set(TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}" CACHE STRING "Target architecture")
if (TARGET_ARCH MATCHES "^arm")
  set(ARM ON) # This means AArch32.
  set(X64 OFF)
  message(STATUS "Building for ARM 32")
elseif (TARGET_ARCH MATCHES "^aarch64")
  set(ARM ON)
  set(X64 ON)
  message(STATUS "Building for ARM 64")
else ()
  set(ARM OFF)
  message(STATUS "Building for x86|x64")
endif ()


if(NOT ARM)

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
      tf sensor_msgs tf_conversions nortek_dvl
      # cv_bridge
      )

    catkin_package(
      INCLUDE_DIRS 
      CATKIN_DEPENDS
        roscpp rospy nav_msgs geometry_msgs
        tf sensor_msgs tf_conversions nortek_dvl
        # cv_bridge
    )

    include_directories(
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      # ${OpenCV_INCLUDE_DIRS}
    )

    # message(STATUS "    testestestestest: $${PROJECT_SOURCE_DIR}")

    list(APPEND thirdparty_libraries
      ${catkin_LIBRARIES}
      # ${OpenCV_LIBRARIES}
    )


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

    add_executable(tools_remap_dvl tools/cpp/remap_dvl.cpp)
    target_link_libraries(tools_remap_dvl
      ${thirdparty_libraries}
    )

    # test
    add_executable(path_node test/path_pub.cpp)
    target_link_libraries(path_node
      ${thirdparty_libraries}
    )

    add_executable(eigen_node test/eigen_to_tf.cpp)
    target_link_libraries(eigen_node
      ${thirdparty_libraries}
    )

else ()
  message(STATUS "Skip build this on the onboard computer")

endif()
