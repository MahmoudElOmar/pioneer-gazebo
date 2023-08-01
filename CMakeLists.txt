cmake_minimum_required(VERSION 2.8.3)
project(ros_publisher)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin")

add_executable(publisher src/vel_publisher.cpp)
target_link_libraries(publisher ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})