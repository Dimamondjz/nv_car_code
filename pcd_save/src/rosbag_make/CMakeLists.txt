cmake_minimum_required(VERSION 3.5)
project(rosbag_make LANGUAGES CXX)
 
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  roscpp
  rospy
  tf
  nav_msgs
  std_msgs
  turtlesim
  message_generation
)
 
generate_messages(
  DEPENDENCIES
  std_msgs
)
 
catkin_package(
   
   CATKIN_DEPENDS 
   geometry_msgs 
   roscpp rospy 
   std_msgs 
   turtlesim 
   message_runtime 
   tf 
   nav_msgs
   LIBRARIES map_filter
   DEPENDS system_lib PCL
)
 
include_directories(include ${catkin_INCLUDE_DIRS} common "/usr/local/include/eigen3" ${PCL_INCLUDE_DIRS})
 
add_executable(pcd_get src/pcd_get.cpp
                include/pcd_grid_divider.h
                src/pcd_grid_divider.cpp
                common/macro.h
                common/util.h
                common/util.cpp
                )
target_link_libraries( pcd_get ${catkin_LIBRARIES} ${PCL_LIBRARIES}  )
