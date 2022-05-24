cmake_minimum_required(VERSION 2.8.3)
project(car_tf_broadcaster)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  roscpp
  tf
  tf2_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
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

add_executable(car_tf_broadcaster
    src/car_tf_broadcaster.cpp
)

target_link_libraries(car_tf_broadcaster
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(car_tf_broadcaster
  ${catkin_EXPORTED_TARGETS}
)
