cmake_minimum_required(VERSION 3.5)
project(orne_box_slam)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})
