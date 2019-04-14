cmake_minimum_required(VERSION 2.8.3)
project(tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
cv_bridge 
image_transport 
camera_info_manager 
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
)


add_executable(save_avi
  src/save_avi.cpp
)
add_dependencies(save_avi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_avi
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)


add_executable(cut_avi
  src/cut_avi.cpp
)
add_dependencies(cut_avi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cut_avi
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)