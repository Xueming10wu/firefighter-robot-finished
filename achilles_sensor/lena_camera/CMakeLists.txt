cmake_minimum_required(VERSION 2.8.3)
project(lena_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package( OpenCV REQUIRED )

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(
  include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
endif()


catkin_package()


include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(lena_ros
  src/lena_ros.cpp
)
add_dependencies(lena_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lena_ros
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)