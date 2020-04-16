cmake_minimum_required(VERSION 2.8.3)
project(rbcar_control)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
