cmake_minimum_required(VERSION 3.0.2)
project(ebot_navigation)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  move_base_msgs
  std_msgs
  actionlib
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



