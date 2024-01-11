cmake_minimum_required(VERSION 3.5)

project(fibonacci_action_client LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} main.c)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  example_interfaces
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
