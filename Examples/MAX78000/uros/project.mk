# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Add your config here!

LIB_FREERTOS = 1

# Set up include paths for uros
IPATH += uros/include
IPATH += uros/include/action_msgs
IPATH += uros/include/actionlib_msgs
IPATH += uros/include/builtin_interfaces
IPATH += uros/include/composition_interfaces
IPATH += uros/include/diagnostic_msgs
IPATH += uros/include/example_interfaces
IPATH += uros/include/geometry_msgs
IPATH += uros/include/libyaml_vendor
IPATH += uros/include/lifecycle_msgs
IPATH += uros/include/micro_ros_msgs
IPATH += uros/include/micro_ros_utilities
IPATH += uros/include/nav_msgs
IPATH += uros/include/rcl
IPATH += uros/include/rcl_action
IPATH += uros/include/rcl_interfaces
IPATH += uros/include/rcl_lifecycle
IPATH += uros/include/rcl_logging_interface
IPATH += uros/include/rclc
IPATH += uros/include/rclc_lifecycle
IPATH += uros/include/rclc_parameter
IPATH += uros/include/rcutils
IPATH += uros/include/rmw
IPATH += uros/include/rmw_microros
IPATH += uros/include/rmw_microxrcedds_c
IPATH += uros/include/rosgraph_msgs
IPATH += uros/include/rosidl_runtime_c
IPATH += uros/include/rosidl_typesupport_c
IPATH += uros/include/rosidl_typesupport_interface
IPATH += uros/include/rosidl_typesupport_introspection_c
IPATH += uros/include/rosidl_typesupport_microxrcedds_c
IPATH += uros/include/sensor_msgs
IPATH += uros/include/shape_msgs
IPATH += uros/include/statistics_msgs
IPATH += uros/include/std_msgs
IPATH += uros/include/std_srvs
IPATH += uros/include/stereo_msgs
IPATH += uros/include/test_msgs
IPATH += uros/include/tracetools
IPATH += uros/include/trajectory_msgs
IPATH += uros/include/ucdr
IPATH += uros/include/unique_identifier_msgs
IPATH += uros/include/uxr
IPATH += uros/include/visualization_msgs

# Add uros static file to the build
PROJ_LDFLAGS += -Luros # Search directory
PROJ_LIBS += microros # Library name

DEBUG = 1
MXC_OPTIMIZE_CFLAGS = -Og

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif
