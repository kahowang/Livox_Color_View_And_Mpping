cmake_minimum_required(VERSION 2.8)
project(livox_color)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Eigen
include_directories("/usr/include/eigen3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  std_srvs
  message_generation 
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport 
)

find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
  save_pose.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS
 INCLUDE_DIRS
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# includels
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(livox_color_view livox_color_view.cpp)
target_link_libraries(livox_color_view ${PCL_LIBRARIES}  ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(livox_color_mapping livox_color_mapping.cpp)
target_link_libraries(livox_color_mapping ${PCL_LIBRARIES}  ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(livox_color_mapping_trigger livox_color_mapping_trigger.cpp)
target_link_libraries(livox_color_mapping_trigger ${PCL_LIBRARIES}  ${catkin_LIBRARIES}  ${OpenCV_LIBS})
