cmake_minimum_required(VERSION 3.0.2)
project(smb_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package( )

install(DIRECTORY launch rviz urdf resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
