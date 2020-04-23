cmake_minimum_required(VERSION 2.8.3)
project(robotic_arm_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
    controller_manager 
  joint_state_controller 
  robot_state_publisher 
)
 find_package(gazebo REQUIRED)
 include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
)

