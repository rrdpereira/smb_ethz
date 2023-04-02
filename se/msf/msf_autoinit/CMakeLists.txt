cmake_minimum_required(VERSION 2.8.3)
project(msf_autoinit)

set(CMAKE_BUILD_TYPE Release)

set(PACKAGE_DEPENDENCIES
    nav_msgs
    rospy
    sensor_fusion_comm
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

catkin_install_python(PROGRAMS scripts/msf_autoinit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)