# cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 3.5)
project(airsim_utils)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   sensor_msgs
#   image_transport
# )

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# catkin_package(
#   CATKIN_DEPENDS
#   sensor_msgs
#   image_transport
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)


# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

add_executable(depth_image_filter src/depth_image_filter.cpp)
ament_target_dependencies(depth_image_filter rclcpp std_msgs sensor_msgs image_transport)
# target_link_libraries(depth_image_filter ${catkin_LIBRARIES})


# catkin_install_python(PROGRAMS
#   scripts/airsim_bridge.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS
  depth_image_filter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/airsim_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()