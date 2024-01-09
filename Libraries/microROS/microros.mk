##############################################################################
 #
 # Copyright 2023 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################

# If CAMERA_DIR is not specified, this Makefile will locate itself.
ifeq "$(MICROROS_DIR)" ""
MICROROS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

# Select the ROS version.  Currently, ROS Humble Hawksbill is the only supported version
# https://docs.ros.org/en/humble/index.html
# Note: "ROS_DISTRO" will be set in the environment for most ROS setups.
ROS_DISTRO ?= humble

# Select the RTOS backend.  Currently, FreeRTOS is the only supported backend.
# In the future, Zephyr support can be selected here.  "Libraries/libs.mk" is responsible
# for checking the value of MICROROS_RTOS and enabling the correct library.
MICROROS_RTOS ?= FREERTOS

# ROS offers multiple "distros".  We will want to maintain some ability to support multiple,
# and eventually archive deprecated distros.  Each distro should get its own folder so
# we can use the "ROS_DISTRO" variable to select the appropriate library file.
MICROROS_LIB_DIR := $(MICROROS_DIR)/lib/$(ROS_DISTRO)
ifeq "$(wildcard $(MICROROS_LIB_DIR))" ""
$(error Failed to load microROS library.  Unsupported ROS version '$(ROS_DISTRO)')
endif

# Add platform implementation.  The top-level "platform" directory should contain the header
# files to implement.  Currently, that's just the serial transport function definitions.
# Implementations should be specific to an RTOS layer (not a microcontroller!).
IPATH += $(MICROROS_DIR)/src/platform

ifeq "$(MICROROS_RTOS)" "FREERTOS"
# Handle FreeRTOS implementation
IPATH += $(MICROROS_DIR)/src/platform/FreeRTOS
VPATH += $(MICROROS_DIR)/src/platform/FreeRTOS
SRCS += mxc_transports.c
SRCS += main.c
FREERTOS_CONFIG_DIR = $(MICROROS_DIR)/src/platform/FreeRTOS
else
$(error Failed to load microROS library.  Unsupported RTOS '$(MICROROS_RTOS)')
endif

# Set up include paths for uros
IPATH += $(MICROROS_LIB_DIR)/include
IPATH += $(MICROROS_LIB_DIR)/include/action_msgs
IPATH += $(MICROROS_LIB_DIR)/include/actionlib_msgs
IPATH += $(MICROROS_LIB_DIR)/include/builtin_interfaces
IPATH += $(MICROROS_LIB_DIR)/include/composition_interfaces
IPATH += $(MICROROS_LIB_DIR)/include/diagnostic_msgs
IPATH += $(MICROROS_LIB_DIR)/include/example_interfaces
IPATH += $(MICROROS_LIB_DIR)/include/geometry_msgs
IPATH += $(MICROROS_LIB_DIR)/include/libyaml_vendor
IPATH += $(MICROROS_LIB_DIR)/include/lifecycle_msgs
IPATH += $(MICROROS_LIB_DIR)/include/micro_ros_msgs
IPATH += $(MICROROS_LIB_DIR)/include/micro_ros_utilities
IPATH += $(MICROROS_LIB_DIR)/include/micro_ros_utilities
IPATH += $(MICROROS_LIB_DIR)/include/nav_msgs
IPATH += $(MICROROS_LIB_DIR)/include/rcl
IPATH += $(MICROROS_LIB_DIR)/include/rcl_action
IPATH += $(MICROROS_LIB_DIR)/include/rcl_interfaces
IPATH += $(MICROROS_LIB_DIR)/include/rcl_lifecycle
IPATH += $(MICROROS_LIB_DIR)/include/rcl_logging_interface
IPATH += $(MICROROS_LIB_DIR)/include/rclc
IPATH += $(MICROROS_LIB_DIR)/include/rclc_lifecycle
IPATH += $(MICROROS_LIB_DIR)/include/rclc_parameter
IPATH += $(MICROROS_LIB_DIR)/include/rcutils
IPATH += $(MICROROS_LIB_DIR)/include/rmw
IPATH += $(MICROROS_LIB_DIR)/include/rmw_microros
IPATH += $(MICROROS_LIB_DIR)/include/rmw_microxrcedds_c
IPATH += $(MICROROS_LIB_DIR)/include/rosgraph_msgs
IPATH += $(MICROROS_LIB_DIR)/include/rosidl_runtime_c
IPATH += $(MICROROS_LIB_DIR)/include/rosidl_typesupport_c
IPATH += $(MICROROS_LIB_DIR)/include/rosidl_typesupport_interface
IPATH += $(MICROROS_LIB_DIR)/include/rosidl_typesupport_introspection_c
IPATH += $(MICROROS_LIB_DIR)/include/rosidl_typesupport_microxrcedds_c
IPATH += $(MICROROS_LIB_DIR)/include/sensor_msgs
IPATH += $(MICROROS_LIB_DIR)/include/shape_msgs
IPATH += $(MICROROS_LIB_DIR)/include/statistics_msgs
IPATH += $(MICROROS_LIB_DIR)/include/std_msgs
IPATH += $(MICROROS_LIB_DIR)/include/std_srvs
IPATH += $(MICROROS_LIB_DIR)/include/stereo_msgs
IPATH += $(MICROROS_LIB_DIR)/include/test_msgs
IPATH += $(MICROROS_LIB_DIR)/include/tracetools
IPATH += $(MICROROS_LIB_DIR)/include/trajectory_msgs
IPATH += $(MICROROS_LIB_DIR)/include/ucdr
IPATH += $(MICROROS_LIB_DIR)/include/unique_identifier_msgs
IPATH += $(MICROROS_LIB_DIR)/include/uxr
IPATH += $(MICROROS_LIB_DIR)/include/visualization_msgs

# Add uros static file to the build.
PROJ_LDFLAGS += -L$(MICROROS_LIB_DIR)
PROJ_LIBS += microros

PROJ_CFLAGS += -D_POSIX_TIMERS

$(info Library enabled: microROS (targeting ROS $(ROS_DISTRO), using $(MICROROS_RTOS)))
