cmake_minimum_required(VERSION 3.5)

project(ping_uros_agent LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME} main.c)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  rmw_microxrcedds
  std_msgs
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
