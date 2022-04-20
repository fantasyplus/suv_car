cmake_minimum_required(VERSION 3.0.2)
project(xavier_upstart)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospack
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#gps_saver
add_executable(gps_saver
    src/gps_saver.cpp
)

target_link_libraries(gps_saver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(gps_saver
  ${catkin_EXPORTED_TARGETS}
)
