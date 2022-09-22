cmake_minimum_required(VERSION 2.8.3)
project(orne_box_setup)
find_package(catkin REQUIRED)
catkin_package()
install(FILES
    scripts/create_udev_rules
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(FILES
    config/70-sensors.rules
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

