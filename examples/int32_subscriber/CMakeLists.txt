cmake_minimum_required(VERSION 3.5)

project(int32_subscriber)

set(MICROROS_DIR "${CMAKE_SOURCE_DIR}/firmware/build" CACHE STRING "micro-ROS build directory")
set(MICROROS_INCLUDE_DIR ${MICROROS_DIR}/include)
set(MICROROS_LIBRARY_DIR ${MICROROS_DIR})
message("MICROROS_DIR: ${MICROROS_DIR}")

include_directories(
  ${MICROROS_INCLUDE_DIR}
  ${MICROROS_INCLUDE_DIR}/rcl
  ${MICROROS_INCLUDE_DIR}/rcl_action
  ${MICROROS_INCLUDE_DIR}/rcutils
  ${MICROROS_INCLUDE_DIR}/rmw
  ${MICROROS_INCLUDE_DIR}/rosidl_runtime_c
  ${MICROROS_INCLUDE_DIR}/rosidl_typesupport_interface

  ${MICROROS_INCLUDE_DIR}/action_msgs
  ${MICROROS_INCLUDE_DIR}/builtin_interfaces
  ${MICROROS_INCLUDE_DIR}/std_msgs
  ${MICROROS_INCLUDE_DIR}/unique_identifier_msgs
)

link_directories(
  ${MICROROS_LIBRARY_DIR}
)

link_libraries(
  microros
)

add_executable(${PROJECT_NAME} main.cpp)
