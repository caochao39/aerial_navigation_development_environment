# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5)
project(control_tuner)

# set(CMAKE_BUILD_TYPE Release)
# set(BUILD_STATIC_LIBS ON)
# set(BUILD_SHARED_LIBS OFF)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   sensor_msgs
# )
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS
#   roscpp
#   std_msgs
#   sensor_msgs
# )

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   "${PROJECT_SOURCE_DIR}/include"
#   /usr/local/include # Location when using 'make system_install'
#   /usr/include       # More usual location (e.g. when installing using a package)
# )

# ## Specify additional locations for library files
# link_directories(
#   /usr/local/lib # Location when using 'make system_install'
#   /usr/lib       # More usual location (e.g. when installing using a package)
# )

## Declare executables
# add_executable(controlTuner src/controlTuner.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(controlTuner ${catkin_LIBRARIES})

add_executable(controlTuner src/controlTuner.cpp)
ament_target_dependencies(controlTuner rclcpp std_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs tf2)

# install(TARGETS controlTuner
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

install(TARGETS
  controlTuner
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
