cmake_minimum_required(VERSION 3.0.2)
project(xihelm_turtlebot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NodeStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xihelm_turtlebot
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

roslint_python()

# set(_RUNLINT "${CMAKE_CURRENT_SOURCE_DIR}/configs/runlint.py")
# roslint_custom(${_RUNLINT} 16 pylint --rcfile=${CMAKE_CURRENT_SOURCE_DIR}/configs/.pylintrc ${CMAKE_CURRENT_SOURCE_DIR}/scripts/xihelm_node.py ${CMAKE_CURRENT_SOURCE_DIR}/scripts/xihelm_controller.py ${CMAKE_CURRENT_SOURCE_DIR}/scripts/xihelm_pid.py)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest(test/xihelm_node_utest.launch)
endif()

