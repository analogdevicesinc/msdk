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
# $Id: Makefile_arm_cm4f 22023 2016-03-21 20:41:55Z zach.metzinger $
#
################################################################################

# This is the name of the build output file
override PROJECT=librtos

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

ifeq "$(BUILD_DIR)" ""
BUILD_DIR=./Build
endif

# This is the path to the CMSIS root directory
ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../CMSIS
endif

# This is the path to the CMSIS root directory
ifeq "$(RTOS_CONFIG_DIR)" ""
RTOS_CONFIG_DIR=.
endif

# Source files for this test (add path to VPATH below)
SRCS  = list.c
SRCS += queue.c
SRCS += tasks.c
SRCS += port.c
SRCS += heap_2.c
SRCS += timers.c
SRCS += croutine.c
SRCS += event_groups.c
SRCS += stream_buffer.c

# Where to find source files for this project
VPATH  = Source/portable/GCC/ARM_CM4F
VPATH += Source/portable/MemMang
VPATH += Source

# Where to find header files for this project
IPATH  = $(RTOS_CONFIG_DIR)
IPATH += Source/portable/GCC/ARM_CM4F
IPATH += .
IPATH += Source/include

# Include the rules for building for this target
TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL ?= lib
