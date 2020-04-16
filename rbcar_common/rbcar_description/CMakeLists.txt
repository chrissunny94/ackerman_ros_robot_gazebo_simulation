cmake_minimum_required(VERSION 2.8.3)
project(rbcar_description)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  std_msgs
  std_srvs
  tf
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
