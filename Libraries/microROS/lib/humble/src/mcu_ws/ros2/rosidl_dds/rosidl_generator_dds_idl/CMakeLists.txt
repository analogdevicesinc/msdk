cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_dds_idl NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(pytest test)
endif()

ament_package(
  CONFIG_EXTRAS "rosidl_generator_dds_idl-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_dds_idl
  DESTINATION lib/rosidl_generator_dds_idl
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
