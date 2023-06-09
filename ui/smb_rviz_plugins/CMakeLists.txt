cmake_minimum_required(VERSION 3.0.2)
project(smb_rviz_plugins)
find_package(catkin REQUIRED COMPONENTS 
  rviz 
  sensor_msgs 
  std_msgs
  roscpp
  rospy
  smb_powerstatus
)

link_directories(${catkin_LIBRARY_DIRS} )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

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
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SRC_FILES
  # Motor power
  src/smb_power_motor/smb_pow_mot_display.cpp
  src/smb_power_motor/smb_batt_mot_panel.cpp
  # Payload power
  src/smb_power_payload/smb_batt_pay_panel.cpp
  src/smb_power_payload/smb_plug_pay_panel.cpp
  src/smb_power_payload/smb_pow_pay_display.cpp
  # Control
  src/smb_control/smb_control_panel.cpp
)

set(HDR_FILES
  # Motor power 
  include/smb_power_motor/smb_pow_mot_display.hpp
  include/smb_power_motor/smb_batt_mot_panel.hpp
  # Payload power
  include/smb_power_payload/smb_batt_pay_panel.hpp
  include/smb_power_payload/smb_plug_pay_panel.hpp
  include/smb_power_payload/smb_pow_pay_display.hpp
  # Control
  include/smb_control/smb_control_panel.hpp
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(
  ${PROJECT_NAME} 
  ${SRC_FILES} 
  ${HDR_FILES}
  resources/resources.qrc
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)