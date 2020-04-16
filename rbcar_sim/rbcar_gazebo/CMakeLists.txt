cmake_minimum_required(VERSION 2.8.3)
project(rbcar_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rbcar_control
  rbcar_description
  roscpp
  std_msgs
  std_srvs
  tf
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
