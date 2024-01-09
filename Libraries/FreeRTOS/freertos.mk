###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
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

################################################################################
# This file can be included in a project makefile to build the library for the 
# project.
################################################################################

ifeq "$(RTOS_DIR)" ""
# If RTOS_DIR is not specified, this Makefile will locate itself.
RTOS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
RTOS_BUILD_DIR=$(CURDIR)/build/FreeRTOS
else
RTOS_BUILD_DIR=$(BUILD_DIR)/FreeRTOS
endif

# Export paths needed by the peripheral driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
RTOS_BUILD_DIR := ${abspath ${RTOS_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export RTOS_CONFIG_DIR := ${abspath ${RTOS_CONFIG_DIR}}

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS

# Add to library list
LIBS += ${RTOS_BUILD_DIR}/librtos.a

# Add to include directory list
IPATH += $(RTOS_CONFIG_DIR)
IPATH += ${RTOS_DIR}
IPATH += ${RTOS_DIR}/Source/portable/$(COMPILER)/ARM_CM4F
IPATH += ${RTOS_DIR}/Source/include

# Add rule to build the Driver Library
${RTOS_BUILD_DIR}/librtos.a: FORCE
	$(MAKE) -C ${RTOS_DIR} lib BUILD_DIR=${RTOS_BUILD_DIR}

