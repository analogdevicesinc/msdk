cmake_minimum_required(VERSION 3.5)

project(multithread_publisher_subscriber LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(micro_ros_utilities REQUIRED)
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} main.c)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  std_msgs
  rmw_microxrcedds
  micro_ros_utilities
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
