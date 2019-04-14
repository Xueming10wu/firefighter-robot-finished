cmake_minimum_required(VERSION 2.8.3)
project(tr10_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trd_diff_controller_node
    src/trd_diff_controller_node.cpp
    src/trd_diff_controller.cpp
    src/message_manager.cpp
    src/message.cpp
    src/serial_port.cpp
)
add_dependencies(trd_diff_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trd_diff_controller_node
  ${catkin_LIBRARIES}
)

add_executable(get_motor_status
    src/get_motor_status.cpp
    src/message_manager.cpp
    src/message.cpp
    src/serial_port.cpp
)
add_dependencies(get_motor_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_motor_status
  ${catkin_LIBRARIES}
)
