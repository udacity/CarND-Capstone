cmake_minimum_required(VERSION 2.8.3)
project(twist_controller)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/dbw_node
                               scripts/dbw_test
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
