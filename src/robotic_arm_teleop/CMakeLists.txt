cmake_minimum_required(VERSION 2.8.3)
project(robotic_arm_teleop)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  rospy
  std_msgs
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  robotic_teleop_arm_gripper_keyboard
  include/robotic_arm_teleop/robotic_teleop_arm_gripper_keyboard.h
  src/robotic_teleop_arm_gripper_keyboard.cpp
)

target_link_libraries(
  robotic_teleop_arm_gripper_keyboard
  ${catkin_LIBRARIES}
)