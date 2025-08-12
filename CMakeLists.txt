cmake_minimum_required(VERSION 3.0.2)
project(arm100_hardware)

## Compiler settings
add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic -fPIC)

## Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  pluginlib
  controller_manager
  std_srvs
  sensor_msgs
  trajectory_msgs
)

## Package configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES so_arm_100_hardware SCServo
  CATKIN_DEPENDS roscpp hardware_interface pluginlib controller_manager std_srvs sensor_msgs trajectory_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build SCServo library (auto add all .cpp in src/SCServo_Linux)
file(GLOB SCSERVO_SRC
  src/SCServo_Linux/*.cpp
)
add_library(SCServo STATIC ${SCSERVO_SRC})
target_include_directories(SCServo PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/SCServo_Linux
)

## Build main hardware interface library
add_library(${PROJECT_NAME}
  src/so_arm_100_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
  SCServo
  ${catkin_LIBRARIES}
)

## Build hardware node
add_executable(so_arm_100_hw_node src/so_arm_100_hw_node.cpp)
target_link_libraries(so_arm_100_hw_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS ${PROJECT_NAME} SCServo so_arm_100_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Install shared resources
install(DIRECTORY config launch urdf scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install plugin description file
install(FILES so_arm_100_hardware.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
