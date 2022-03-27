cmake_minimum_required(VERSION 3.0.2)
project(airsim_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport
)


catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_image_filter src/depth_image_filter.cpp)
target_link_libraries(depth_image_filter ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  scripts/airsim_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
