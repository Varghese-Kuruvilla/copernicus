cmake_minimum_required(VERSION 2.8.3)
project(copernicus_teleoperator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  copernicus_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    copernicus_joy_teleop
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(copernicus_joy_teleop src/joy_teleop.cpp)

add_executable(copernicus_joy_teleop_node src/joy_teleop_node.cpp)

target_link_libraries(copernicus_joy_teleop_node copernicus_joy_teleop ${catkin_LIBRARIES})