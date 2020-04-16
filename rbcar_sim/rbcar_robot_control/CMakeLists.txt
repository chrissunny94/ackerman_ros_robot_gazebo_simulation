cmake_minimum_required(VERSION 2.8.3)
project(rbcar_robot_control)


find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  tf
  diagnostic_msgs
  diagnostic_updater
)

catkin_package(
 CATKIN_DEPENDS robotnik_msgs ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rbcar_robot_control_node src/rbcar_robot_control.cpp)


add_dependencies(rbcar_robot_control_node rbcar_robot_control_generate_messages_cpp)
add_dependencies(rbcar_robot_control_node robotnik_msgs_generate_messages_cpp)


target_link_libraries(rbcar_robot_control_node
   ${catkin_LIBRARIES}
)
