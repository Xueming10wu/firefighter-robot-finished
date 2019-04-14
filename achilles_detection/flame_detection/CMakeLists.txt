cmake_minimum_required(VERSION 2.8.3)
project(flame_detection)

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

add_executable(main
  src/main.cpp
  src/utils.cpp
  src/FeatureAnalyzer.cpp 
  src/FlameDecider.cpp 
  src/FlameDetector.cpp  
  src/TargetExtractor.cpp  
  src/Handler.cpp
)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(3DPosition
  src/3DPosition.cpp
)
add_dependencies(3DPosition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(3DPosition
  ${catkin_LIBRARIES}
)