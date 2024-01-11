cmake_minimum_required(VERSION 3.5)

project(static_type_handling LANGUAGES C)

set (CMAKE_C_STANDARD 11)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(micro_ros_utilities REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME} main.c)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  sensor_msgs
  visualization_msgs
  rmw_microxrcedds
  micro_ros_utilities
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
