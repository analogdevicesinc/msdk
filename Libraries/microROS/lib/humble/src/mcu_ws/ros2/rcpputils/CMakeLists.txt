cmake_minimum_required(VERSION 3.5)

project(rcpputils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # enables building a static library but later link it into a dynamic library
  add_compile_options(-fPIC)
endif()
if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME}
  src/asserts.cpp
  src/filesystem_helper.cpp
  src/find_library.cpp
  src/env.cpp
  src/shared_library.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "RCPPUTILS_BUILDING_LIBRARY")
endif()
ament_target_dependencies(${PROJECT_NAME} rcutils)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rcutils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wthread-safety -Werror)
  endif()

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_asserts_ndebug test/test_asserts.cpp)
  target_link_libraries(test_asserts_ndebug ${PROJECT_NAME})

  if(TARGET test_asserts_ndebug)
    target_compile_definitions(test_asserts_ndebug PUBLIC NDEBUG)
  endif()

  ament_add_gtest(test_asserts_debug test/test_asserts.cpp)
  target_link_libraries(test_asserts_debug ${PROJECT_NAME})

  ament_add_gtest(test_thread_safety_annotations test/test_thread_safety_annotations.cpp)
  target_link_libraries(test_thread_safety_annotations ${PROJECT_NAME})

  ament_add_gtest(test_join test/test_join.cpp)
  target_link_libraries(test_join ${PROJECT_NAME})

  ament_add_gtest(test_time test/test_time.cpp)
  target_link_libraries(test_time ${PROJECT_NAME})
  ament_target_dependencies(test_time rcutils)

  ament_add_gtest(test_env test/test_env.cpp
    ENV
      EMPTY_TEST=
      NORMAL_TEST=foo
  )
  ament_target_dependencies(test_env rcutils)
  target_link_libraries(test_env ${PROJECT_NAME})

  ament_add_gtest(test_scope_exit test/test_scope_exit.cpp)
  target_link_libraries(test_scope_exit ${PROJECT_NAME})

  ament_add_gtest(test_split test/test_split.cpp)
  target_link_libraries(test_split ${PROJECT_NAME})

  ament_add_gtest(test_filesystem_helper test/test_filesystem_helper.cpp
    ENV
      EXPECTED_WORKING_DIRECTORY=$<SHELL_PATH:${CMAKE_CURRENT_BINARY_DIR}>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_filesystem_helper rcutils)
  target_link_libraries(test_filesystem_helper ${PROJECT_NAME})

  ament_add_gtest(test_find_and_replace test/test_find_and_replace.cpp)
  target_link_libraries(test_find_and_replace ${PROJECT_NAME})

  ament_add_gtest(test_pointer_traits test/test_pointer_traits.cpp)
  target_link_libraries(test_pointer_traits ${PROJECT_NAME})

  ament_add_gtest(test_process test/test_process.cpp)
  target_link_libraries(test_process ${PROJECT_NAME})
  ament_target_dependencies(test_process rcutils)

  set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

  ament_add_gtest(test_shared_library test/test_shared_library.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}"
  )

  if(TARGET test_shared_library)
    add_library(dummy_shared_library test/dummy_shared_library/dummy_shared_library.c)
    if(WIN32)
      # Causes the visibility macros to use dllexport rather than dllimport
      # which is appropriate when building the dll but not consuming it.
      target_compile_definitions(dummy_shared_library PRIVATE "DUMMY_SHARED_LIBRARY_BUILDING_DLL")
    endif()
    target_link_libraries(test_shared_library ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_endian test/test_endian.cpp)
  target_link_libraries(test_endian ${PROJECT_NAME})

  add_library(test_library SHARED test/test_library.cpp)
  target_link_libraries(test_library PUBLIC ${PROJECT_NAME})
  ament_add_gtest(test_find_library test/test_find_library.cpp)
  target_link_libraries(test_find_library test_library)
  set_tests_properties(test_find_library PROPERTIES
    ENVIRONMENT
      "_TEST_LIBRARY_DIR=$<TARGET_FILE_DIR:test_library>;_TEST_LIBRARY=$<TARGET_FILE:test_library>")

  ament_add_gtest(test_clamp test/test_clamp.cpp)
  target_link_libraries(test_clamp ${PROJECT_NAME})

  ament_add_gtest(test_accumulator test/test_accumulator.cpp)
  target_link_libraries(test_accumulator ${PROJECT_NAME})
endif()

ament_package()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
