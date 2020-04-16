cmake_minimum_required(VERSION 2.8.3)
project(rbcar_pad)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
  ackermann_msgs
  std_srvs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rbcar_pad_node src/rbcar_pad.cpp)


add_dependencies(rbcar_pad_node rbcar_pad_generate_messages_cpp)
add_dependencies(rbcar_pad_node robotnik_msgs_generate_messages_cpp)

target_link_libraries(rbcar_pad_node
   ${catkin_LIBRARIES}
)

