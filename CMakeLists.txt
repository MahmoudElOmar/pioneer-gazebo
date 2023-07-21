cmake_minimum_required(VERSION 3.1)
project(vel_publisher)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(publisher src/vel_publisher.cpp)
target_link_libraries(publisher ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
