cmake_minimum_required(VERSION 2.8.3)
project(vel_publisher)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

#include_directories(${tf2_INCLUDE_DIRS})
#include_directories(${tf2_ros_INCLUDE_DIRS})


#find_package(gazebo REQUIRED)
#include_directories(${GAZEBO_INCLUDE_DIRS})

#link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}) #${GAZEBO_CXX_FLAGS}")

#add_library(controller SHARED src/control_plugin.cc)
#target_link_libraries(controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})
#target_link_libraries(controller ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(publisher src/vel_publisher.cpp)
target_link_libraries(publisher ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})

