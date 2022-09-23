cmake_minimum_required(VERSION 3.0.2)
project(orne_box_description)
find_package(catkin REQUIRED COMPONENTS
  xacro
)
catkin_package()
install(DIRECTORY rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

