###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
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

# Specify the project variant.
PERIPH_DRIVER_LIB_FILENAME ?= libPeriphDriver_$(MFLOAT_ABI)

override PROJECT = $(PERIPH_DRIVER_LIB_FILENAME)

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC ?= $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC ?= $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))

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

include ${CMSIS_ROOT}/../PeriphDrivers/$(TARGET_LC)_files.mk
PERIPH_DRIVER_C_FILES += $(PERIPH_DIR)/Source/SYS/mxc_assert.c
PERIPH_DRIVER_C_FILES += $(PERIPH_DIR)/Source/SYS/mxc_delay.c
PERIPH_DRIVER_C_FILES += $(PERIPH_DIR)/Source/SYS/mxc_lock.c
PERIPH_DRIVER_C_FILES += $(PERIPH_DIR)/Source/SYS/nvic_table.c
PERIPH_DRIVER_C_FILES += $(PERIPH_DIR)/Source/SYS/mxc_sys_common.c

PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_assert.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_delay.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/nvic_table.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_lock.c

# # Where to find header files for this project
IPATH += $(PERIPH_DRIVER_INCLUDE_DIR)
SRCS  += $(PERIPH_DRIVER_C_FILES)
SRCS  += $(PERIPH_DRIVER_A_FILES)
VPATH += $(dir $(SRCS))

# Use absolute paths if building within eclipse environment.
ifeq "$(ECLIPSE)" "1"
SRCS := $(abspath $(SRCS))
endif

# Only building libraries.
MAKECMDGOALS=lib

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
# .DEFAULT_GOAL ?= lib
