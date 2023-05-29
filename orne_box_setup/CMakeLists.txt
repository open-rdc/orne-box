cmake_minimum_required(VERSION 3.5)
project(orne_box_setup)

# Find packages in ROS 2
find_package(ament_cmake REQUIRED)

# Create ament index resource (package.xml)
ament_package()

# Install scripts
install(
    PROGRAMS
    scripts/create_udev_rules
    DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(
    FILES
    config/70-sensors.rules
    DESTINATION share/${PROJECT_NAME}/config
)



# cmake_minimum_required(VERSION 2.8.3)
# project(orne_box_setup)
# find_package(catkin REQUIRED)
# catkin_package()
# install(FILES
#     scripts/create_udev_rules
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
# )
# install(FILES
#     config/70-sensors.rules
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )