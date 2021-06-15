cmake_minimum_required(VERSION 2.8.3)

project(lwr4p)

add_definitions(-DLINUX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  DEPENDS
)

SET(SRC_LIST
  src/lwr4p_dynamics.cpp
)

include_directories(
  include
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
  ${catkin_LIBRARIES}
)
