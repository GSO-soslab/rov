cmake_minimum_required(VERSION 2.8.3)

project(rov_onboard)

add_compile_options(-std=c++11)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    )

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS
    roscpp
)

# build urdf
find_package(xacro REQUIRED)
xacro_add_files(${CMAKE_CURRENT_SOURCE_DIR}/urdf/rov.urdf.xacro TARGET rov INORDER)
install(FILES
    urdf/rov.urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})