cmake_minimum_required(VERSION 3.0.2)
project(lm4075e_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lm4075e_control
  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lm4075e_control src/lm4075e_control.cpp)

target_link_libraries(lm4075e_control
  ${catkin_LIBRARIES}
)