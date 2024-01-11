cmake_minimum_required(VERSION 3.5)

project(micro_ros_demos_rclc)

find_package(ament_cmake REQUIRED)

include(ExternalProject)
include(CMakeParseArguments)

if(ANDROID)
    set(CROSS_CMAKE_ARGS
        -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_SYSTEM_VERSION:STRING=${CMAKE_SYSTEM_VERSION}
        -DCMAKE_ANDROID_ARCH_ABI:STRING=${CMAKE_ANDROID_ARCH_ABI}
        -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE:STRING=BOTH
        )
endif()

function(export_executable)
    set(subtree "")
    foreach(arg IN LISTS ARGN)
        set(subtree ${subtree}/${arg})
    endforeach()
    get_filename_component(leaf ${subtree} NAME)
    ExternalProject_Add(${leaf}
        PREFIX
            ${PROJECT_BINARY_DIR}${subtree}
        SOURCE_DIR
            ${PROJECT_SOURCE_DIR}${subtree}
        INSTALL_DIR
            ${PROJECT_BINARY_DIR}/temp_install
        BUILD_ALWAYS 1
        CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
            -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
            -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            ${CROSS_CMAKE_ARGS}
    )
endfunction()

export_executable(graph_introspection graph_visualizer)
export_executable(graph_introspection subscription_count)
export_executable(graph_introspection publisher_count)
export_executable(int32_publisher)
export_executable(int32_subscriber)
export_executable(configuration_example configured_publisher)
export_executable(configuration_example configured_subscriber)
export_executable(addtwoints_server)
export_executable(addtwoints_client)
export_executable(fibonacci_action_server)
export_executable(fibonacci_action_client)
export_executable(int32_multinode)
export_executable(int32_publisher_subscriber)
export_executable(ping_pong)
export_executable(ping_uros_agent)
export_executable(fragmented_publication)
export_executable(fragmented_subscription)
export_executable(complex_msg_publisher)
export_executable(complex_msg_subscriber)
export_executable(timer)
export_executable(string_publisher)
export_executable(string_subscriber)
export_executable(autodiscover_agent)
export_executable(static_type_handling)
export_executable(epoch_synchronization)
export_executable(parameter_server)
# export_executable(configuration_example custom_transports)
export_executable(multithread_publisher_subscriber)

if(EXISTS ${CMAKE_BINARY_DIR}/temp_install/)
    install(
        DIRECTORY
            ${CMAKE_BINARY_DIR}/temp_install/
        DESTINATION
            lib/${PROJECT_NAME}
        USE_SOURCE_PERMISSIONS
        )
endif()

ament_package()
