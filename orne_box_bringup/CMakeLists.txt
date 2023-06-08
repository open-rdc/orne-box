cmake_minimum_required(VERSION 3.5)
project(orne_box_bringup)


find_package(ament_cmake_auto REQUIRED)


ament_auto_find_build_dependencies()

 
# ament_auto_add_library(${PROJECT_NAME} SHARED
#  src/orne_box_bringup_node.cpp)

# add_executable(orne_box_bringup src/orne_box_bringup_node.cpp)
ament_auto_add_executable(icart_mini_driver
          src/icart_mini_driver_node.cpp)

target_link_libraries(icart_mini_driver ypspur)

target_include_directories(icart_mini_driver
       PUBLIC ${ypspur_INCLUDE_DIRS})

# install(FILES scripts/ypspur_coordinator_bridge
#   DESTINATION ${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# install(
#   PROGRAMS script.sh
#   DESTINATION lib/${PROJECT_NAME})

  ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  scripts

)