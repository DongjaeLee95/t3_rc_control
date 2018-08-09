cmake_minimum_required(VERSION 2.8.3)
project(t3_rc_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES t3_rc_control
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/t3_rc_control.cpp
# )

add_executable(t3_rc_control 
	src/t3_rc_control.cpp
	src/t3_rc_control_main.cpp
)
target_link_libraries(t3_rc_control ${catkin_LIBRARIES})


