cmake_minimum_required(VERSION 2.8.3)

project(lwr4p)

add_definitions(-DLINUX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules like armadillo
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/../install/deps/ ${CMAKE_MODULE_PATH} )
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH} )

find_package(catkin REQUIRED
  kdl_parser
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  robo_lib
  math_lib
  thread_lib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

SET(SRC_LIST
  src/robot.cpp
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pthread
  armadillo
  robo_lib
  math_lib
  thread_lib
  ${catkin_LIBRARIES}
)
