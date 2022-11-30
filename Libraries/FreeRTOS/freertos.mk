################################################################################
# Copyright (C) 2014 Maxim Integrated Products, Inc., All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
# OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Except as contained in this notice, the name of Maxim Integrated
# Products, Inc. shall not be used except as stated in the Maxim Integrated
# Products, Inc. Branding Policy.
#
# The mere transfer of this software does not imply any licenses
# of trade secrets, proprietary technology, copyrights, patents,
# trademarks, maskwork rights, or any other form of intellectual
# property whatsoever. Maxim Integrated Products, Inc. retains all
# ownership rights.
#
# $Id: freertos.mk 16493 2015-02-03 20:19:08Z jeremy.brodt $
#
################################################################################

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

