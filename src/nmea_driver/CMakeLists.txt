cmake_minimum_required(VERSION 2.8.3)
project(nmea_driver)

# find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  roscpp
  tf
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

add_executable(nmea_driver
    src/nmea_driver_node.cpp
    src/nmea_parser.cpp
    src/serial_util.cpp
)

target_link_libraries(nmea_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(nmea_driver 
  ${catkin_EXPORTED_TARGETS}
)
