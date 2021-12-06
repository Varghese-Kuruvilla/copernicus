cmake_minimum_required(VERSION 2.8.3)
project(copernicus_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  joint_state_controller
  roscpp
  std_msgs
  copernicus_msgs
  twist_mux
  gazebo_ros_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_navigation_goals
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(halt_vehicle src/halt_vehicle.cpp)
target_link_libraries(halt_vehicle ${catkin_LIBRARIES})


