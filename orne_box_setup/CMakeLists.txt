cmake_minimum_required(VERSION 2.8.3)
project(orne_box_setup)

find_package(catkin REQUIRED)

catkin_package()

install(FILES
    scripts/create_udev_rules
    scripts/create_robot_params
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(FILES
    config/70-sensors.rules
    config/icart-mini.param
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
