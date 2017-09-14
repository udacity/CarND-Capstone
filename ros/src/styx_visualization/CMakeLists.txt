cmake_minimum_required(VERSION 2.8.3)

project(styx_visualization)

find_package(catkin)

catkin_package()

catkin_install_python(PROGRAMS scripts/lane_to_marker
                               scripts/traffic_light_to_marker
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
