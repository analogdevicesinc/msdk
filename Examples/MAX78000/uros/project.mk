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

UROS_DIR := uros

VPATH += src/cnn
IPATH += src/cnn

# Set up include paths for uros
IPATH += $(UROS_DIR)/include
IPATH += $(UROS_DIR)/include/action_msgs
IPATH += $(UROS_DIR)/include/actionlib_msgs
IPATH += $(UROS_DIR)/include/builtin_interfaces
IPATH += $(UROS_DIR)/include/composition_interfaces
IPATH += $(UROS_DIR)/include/diagnostic_msgs
IPATH += $(UROS_DIR)/include/example_interfaces
IPATH += $(UROS_DIR)/include/geometry_msgs
IPATH += $(UROS_DIR)/include/libyaml_vendor
IPATH += $(UROS_DIR)/include/lifecycle_msgs
IPATH += $(UROS_DIR)/include/micro_ros_msgs
IPATH += $(UROS_DIR)/include/micro_ros_utilities
IPATH += $(UROS_DIR)/include/micro_ros_utilities
IPATH += $(UROS_DIR)/include/nav_msgs
IPATH += $(UROS_DIR)/include/rcl
IPATH += $(UROS_DIR)/include/rcl_action
IPATH += $(UROS_DIR)/include/rcl_interfaces
IPATH += $(UROS_DIR)/include/rcl_lifecycle
IPATH += $(UROS_DIR)/include/rcl_logging_interface
IPATH += $(UROS_DIR)/include/rclc
IPATH += $(UROS_DIR)/include/rclc_lifecycle
IPATH += $(UROS_DIR)/include/rclc_parameter
IPATH += $(UROS_DIR)/include/rcutils
IPATH += $(UROS_DIR)/include/rmw
IPATH += $(UROS_DIR)/include/rmw_microros
IPATH += $(UROS_DIR)/include/rmw_microxrcedds_c
IPATH += $(UROS_DIR)/include/rosgraph_msgs
IPATH += $(UROS_DIR)/include/rosidl_runtime_c
IPATH += $(UROS_DIR)/include/rosidl_typesupport_c
IPATH += $(UROS_DIR)/include/rosidl_typesupport_interface
IPATH += $(UROS_DIR)/include/rosidl_typesupport_introspection_c
IPATH += $(UROS_DIR)/include/rosidl_typesupport_microxrcedds_c
IPATH += $(UROS_DIR)/include/sensor_msgs
IPATH += $(UROS_DIR)/include/shape_msgs
IPATH += $(UROS_DIR)/include/statistics_msgs
IPATH += $(UROS_DIR)/include/std_msgs
IPATH += $(UROS_DIR)/include/std_srvs
IPATH += $(UROS_DIR)/include/stereo_msgs
IPATH += $(UROS_DIR)/include/test_msgs
IPATH += $(UROS_DIR)/include/tracetools
IPATH += $(UROS_DIR)/include/trajectory_msgs
IPATH += $(UROS_DIR)/include/ucdr
IPATH += $(UROS_DIR)/include/unique_identifier_msgs
IPATH += $(UROS_DIR)/include/uxr
IPATH += $(UROS_DIR)/include/visualization_msgs

# Add uros static file to the build
PROJ_LDFLAGS += -L$(UROS_DIR) # Search directory
PROJ_LIBS += microros # Library name

DEBUG = 1
MXC_OPTIMIZE_CFLAGS = -Og

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif
