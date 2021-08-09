cmake_minimum_required(VERSION 3.0.0)

project(project_name_)

 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when needed.
set(CMAKE_AUTOUIC ON) # Tell CMake to run uic when needed.
set(CMAKE_AUTORCC ON) # Tell CMake to run rcc when needed

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui PrintSupport REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})
SET(Qt_LIBRARIES
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  math_lib
  io_lib
  robo_lib
  plot_lib
  thread_lib
  gui_lib
  geometry_msgs
  tf
  rviz_lib
  kdl_parser
  robot_state_publisher
  lwr4p
  ur_robot
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs tf
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

set(SRC_LIST

  include/${PROJECT_NAME}/gui/train_slider.cpp
  include/${PROJECT_NAME}/gui/wrench_ctrl_gui.cpp
  include/${PROJECT_NAME}/gui/main_gui.cpp

  src/main.cpp
  src/main_ctrl.cpp
  src/robot_obj_sim.cpp
  src/ur_wrapper.cpp
)

add_executable(${PROJECT_NAME}
  ${SRC_LIST}
  resources/resource.qrc
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
  math_lib
  io_lib
  robo_lib
  plot_lib
  thread_lib
  rviz_lib
  gui_lib
  ${Qt_LIBRARIES}
  pthread
  lwr4p
  ur_robot
)
