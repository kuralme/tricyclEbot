cmake_minimum_required(VERSION 2.8.3)
project(ebot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
)
find_package(gazebo REQUIRED)
find_package(roslaunch)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

foreach(dir launch worlds)
	install(DIRECTORY ${dir}/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
