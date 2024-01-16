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
TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL ?= lib
