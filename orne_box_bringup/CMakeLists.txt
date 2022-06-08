cmake_minimum_required(VERSION 3.0.0)
project(orne_box_bringup VERSION 0.1.0)

find_package(catkin REQUIRED
  roscpp
  tf2_ros
  param_io
  geometry_msgs
)

catkin_package()

add_executable(projected_transform_publisher_node
  src/projected_transform_publisher_node.cpp
)

target_compile_features(projected_transform_publisher_node
  PRIVATE
    cxx_std_14
)

target_include_directories(projected_transform_publisher_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(projected_transform_publisher_node
  ${catkin_LIBRARIES}
)

