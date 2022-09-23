cmake_minimum_required(VERSION 3.0.2)
project(orne_box_simulations)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)
find_package(gazebo REQUIRED)
catkin_package(
)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
