cmake_minimum_required(VERSION 2.8.3)
project(orne_box_strategy)

find_package(catkin REQUIRED COMPONENTS
  fulanghua_srvs
  smach
  smach_ros
  std_srvs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
