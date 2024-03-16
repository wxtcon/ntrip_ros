cmake_minimum_required(VERSION 3.0.2)

project(ntrip_client)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gnss_comm
)


include_directories(
  include/ntrip_client
  src
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trigger
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_library(nc_utils SHARED src/ntrip_util.cpp)

add_executable(ntrip_client src/main.cpp)

target_link_libraries(ntrip_client ${catkin_LIBRARIES})
target_link_libraries(ntrip_client  nc_utils)