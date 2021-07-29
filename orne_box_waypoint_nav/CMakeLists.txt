cmake_minimum_required(VERSION 3.0.2)
project(orne_box_waypoint_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  interactive_markers
  tf
  tf2
  tf2_ros
  rviz
  std_srvs
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy sensor_msgs std_msgs tf tf2 tf2_ros yaml-cpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

set(SRC_FILES
  src/save_trigger_panel.cpp
  src/state_trigger_panel.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(
  ${PROJECT_NAME} 
  ${QT_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_executable(waypoint_editor src/waypoint_editor.cpp)
target_link_libraries(
    waypoint_editor
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)

add_executable(waypoint_navigator src/waypoint_navigator.cpp)
target_link_libraries(
    waypoint_navigator
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)

add_executable(waypoint_saver src/waypoint_saver.cpp)
target_link_libraries(
    waypoint_saver
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )

install(FILES 
plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)