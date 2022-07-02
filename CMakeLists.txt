cmake_minimum_required(VERSION 3.10.2)
project(motor_control)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  yaml-cpp
)

add_subdirectory(our_hw_sdk)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorControlCmd.msg
  MotorState.msg
  MotorCommonMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MaxSpeed.srv
  MotorConfig.srv
  ManagerConfig.srv
  MotorCommon.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motor_control
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_node src/motor_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(motor_node ${catkin_LIBRARIES} our_hw_sdk)

add_executable(motor_config_server src/motor_config_service.cpp)
target_link_libraries(motor_config_server ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(motor_config_server ${PROJECT_NAME}_gencpp)
