
cmake_minimum_required(VERSION 2.8.3)
project(quadcopter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs geometry_msgs message_generation)


add_message_files(
  FILES
  Thrust.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nodelet_quad
  CATKIN_DEPENDS nodelet roscpp std_msgs message_generation geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Create the nodelet tutorial library
add_library(nodelet_quad src/Quadcopter.cpp)
target_link_libraries(nodelet_quad ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_quad ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS nodelet_quad
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_quad.xml quacopter_params.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})