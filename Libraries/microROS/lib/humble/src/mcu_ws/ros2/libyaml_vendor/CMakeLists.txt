cmake_minimum_required(VERSION 3.9)

project(libyaml_vendor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # About -Wno-sign-conversion: With Clang, -Wconversion implies -Wsign-conversion. There are a number of
  # implicit sign conversions in rclcpp and gtest.cc, see https://ci.ros2.org/job/ci_osx/9265/.
  # Hence disabling -Wsign-conversion for now until all those have eventually been fixed.
  # (from https://github.com/ros2/rclcpp/pull/1188#issuecomment-650229140)
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)

macro(build_libyaml)
  set(extra_cmake_args)

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  list(APPEND extra_cmake_args -DBUILD_SHARED_LIBS=ON)
  list(APPEND extra_cmake_args -DBUILD_TESTING=OFF)
  list(APPEND extra_cmake_args -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS})
  # Disable MSVC warnings
  if(CMAKE_C_COMPILER_ID MATCHES "MSVC")
    set(win_c_flags "${CMAKE_C_FLAGS} /wd4244 /wd4267 /wd4996")
    list(APPEND extra_cmake_args -DCMAKE_C_FLAGS=${win_c_flags})
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
    if(ANDROID)
      if(DEFINED ANDROID_ABI)
        list(APPEND extra_cmake_args "-DANDROID_ABI=${ANDROID_ABI}")
      endif()
      if(DEFINED ANDROID_CPP_FEATURES)
        list(APPEND extra_cmake_args "-DANDROID_CPP_FEATURES=${ANDROID_CPP_FEATURES}")
      endif()
      if(DEFINED ANDROID_FUNCTION_LEVEL_LINKING)
        list(APPEND extra_cmake_args "-DANDROID_FUNCTION_LEVEL_LINKING=${ANDROID_FUNCTION_LEVEL_LINKING}")
      endif()
      if(DEFINED ANDROID_NATIVE_API_LEVEL)
        list(APPEND extra_cmake_args "-DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL}")
      endif()
      if(DEFINED ANDROID_NDK)
        list(APPEND extra_cmake_args "-DANDROID_NDK=${ANDROID_NDK}")
      endif()
      if(DEFINED ANDROID_STL)
        list(APPEND extra_cmake_args "-DANDROID_STL=${ANDROID_STL}")
      endif()
      if(DEFINED ANDROID_TOOLCHAIN_NAME)
        list(APPEND extra_cmake_args "-DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME}")
      endif()
    endif()
  endif()
  include(ExternalProject)
  externalproject_add(libyaml-0.2.5
    GIT_REPOSITORY https://github.com/yaml/libyaml.git
    GIT_TAG 2c891fc7a770e8ba2fec34fc6b545c672beb37e6  # 0.2.5
    GIT_CONFIG advice.detachedHead=false
    # Suppress git update due to https://gitlab.kitware.com/cmake/cmake/-/issues/16419
    # See https://github.com/ament/uncrustify_vendor/pull/22 for details
    UPDATE_COMMAND ""
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libyaml_install
      -DINSTALL_INCLUDE_DIR:STRING=include/${PROJECT_NAME}
      ${extra_cmake_args}
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E chdir <SOURCE_DIR> git apply -p1 --ignore-space-change --whitespace=nowarn
        ${CMAKE_CURRENT_SOURCE_DIR}/0001-Install-yaml.h-to-INSTALL_INCLUDE_DIR.patch
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/libyaml_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
    PATTERN config.h EXCLUDE
  )

  set(yaml_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/libyaml_install/include/${PROJECT_NAME})
  set(yaml_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}/libyaml_install/lib)
  set(yaml_LIBRARIES yaml)
endmacro()

build_libyaml()
set(extra_test_dependencies libyaml-0.2.5)

ament_export_libraries(yaml)
ament_export_dependencies(yaml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(performance_test_fixture REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS
    performance_test_fixture::performance_test_fixture INTERFACE_INCLUDE_DIRECTORIES)

  if(${CMAKE_VERSION} VERSION_LESS "3.13")
    link_directories(${yaml_LIBRARY_DIRS})
  endif()

  ament_add_gtest(test_yaml_reader test/test_reader.cpp
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  if(TARGET test_yaml_reader)
    target_include_directories(test_yaml_reader PRIVATE ${yaml_INCLUDE_DIRS})
    target_link_libraries(test_yaml_reader ${yaml_LIBRARIES})
    if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13")
      target_link_directories(test_yaml_reader PRIVATE ${yaml_LIBRARY_DIRS})
    endif()
    ament_target_dependencies(test_yaml_reader rcpputils)
    if(extra_test_dependencies)
      add_dependencies(test_yaml_reader ${extra_test_dependencies})
    endif()
  endif()

  add_performance_test(benchmark_reader test/benchmark/benchmark_reader.cpp
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  ament_target_dependencies(benchmark_reader rcpputils)
  if(TARGET benchmark_reader)
    target_include_directories(benchmark_reader PRIVATE ${yaml_INCLUDE_DIRS})
    target_link_libraries(benchmark_reader ${yaml_LIBRARIES})
    if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13")
      target_link_directories(benchmark_reader PRIVATE ${yaml_LIBRARY_DIRS})
    endif()
    if(extra_test_dependencies)
      add_dependencies(benchmark_reader ${extra_test_dependencies})
    endif()
  endif()
endif()

ament_package(CONFIG_EXTRAS libyaml_vendor-extras.cmake)
