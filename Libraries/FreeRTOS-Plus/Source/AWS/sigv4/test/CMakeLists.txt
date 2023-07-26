# Project information.
cmake_minimum_required( VERSION 3.13.0 )
project( "SigV4 unit test"
          LANGUAGES C )

# Allow the project to be organized into folders.
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

set( CMAKE_C_STANDARD_REQUIRED ON )

# Do not allow in-source build.
if( ${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR} )
    message( FATAL_ERROR "In-source build is not allowed. Please build in a separate directory, such as ${PROJECT_SOURCE_DIR}/build." )
endif()

# Set global path variables.
get_filename_component( __MODULE_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE )
set( MODULE_ROOT_DIR ${__MODULE_ROOT_DIR} CACHE INTERNAL "SigV4 repository root." )

# Configure options to always show in CMake GUI.
option( BUILD_UNIT_TESTS
        "Set this to ON to enable unit tests. If the submodule for CMock testing framework is not cloned, it will be auto-cloned."
        OFF )

# Set output directories.
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

#  ====================== Coverity Analysis Configuration ======================
# Include filepaths for source and include.
include( ${MODULE_ROOT_DIR}/sigv4FilePaths.cmake )

# Target for Coverity analysis that builds the library.
add_library( coverity_analysis
            ${SIGV4_SOURCES} )

# Verify ISO C90 compliance of libray.
target_compile_options( coverity_analysis PUBLIC -std=c90 )

# SigV4 public include path and test config file
target_include_directories( coverity_analysis
                            PUBLIC
                            ${SIGV4_INCLUDE_PUBLIC_DIRS}
                            "${CMAKE_CURRENT_LIST_DIR}/include" )

# Build without debug enabled when performing static analysis
target_compile_options(coverity_analysis PUBLIC -DNDEBUG -DDISABLE_LOGGING)

#  ============================  Test Configuration ============================
if(${BUILD_UNIT_TESTS})
    # Define a CMock resource path.
    set( CMOCK_DIR ${MODULE_ROOT_DIR}/test/unit-test/CMock CACHE INTERNAL "CMock library source directory." )

    # Include CMock build configuration.
    include( unit-test/cmock_build.cmake )

    # Check if the CMock source directory exists, and if not present, clone the submodule
    # if BUILD_CLONE_SUBMODULES configuration is enabled.
    if( NOT EXISTS ${CMOCK_DIR}/src )
        # Attempt to clone CMock.
        clone_cmock()
    endif()

    # Add unit test and coverage configuration.

    # Use CTest utility for managing test runs. This has to be added BEFORE
    # defining test targets with add_test()
    enable_testing()

    # Add build targets for CMock and Unit, required for unit testing.
    add_cmock_targets()

    # Add function to enable CMock based tests and coverage.
    include( ${MODULE_ROOT_DIR}/tools/cmock/create_test.cmake )

    # Include build configuration for unit tests.
    add_subdirectory( unit-test )

    #  ==================== Coverage Analysis configuration ========================

    # Add a target for running coverage on tests.
    add_custom_target( coverage
        COMMAND ${CMAKE_COMMAND} -DCMOCK_DIR=${CMOCK_DIR}
        -P ${MODULE_ROOT_DIR}/tools/cmock/coverage.cmake
        DEPENDS cmock unity sigv4_utest
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()
