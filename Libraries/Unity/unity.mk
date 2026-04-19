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

################################################################################
# This file can be included in a project makefile to build the library for the 
# project.
###############################################################################

ifeq "$(LIB_UNITY_DIR)" ""
# If UNITY_DIR is not specified, this Makefile will locate itself.
LIB_UNITY_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

ifeq "$(PROJECT)" ""
PROJECT = $(TARGET_LC)
endif

# Detect current OS.
include $(MAXIM_PATH)/Libraries/CMSIS/Device/Maxim/GCC/detect_os.mk

IPATH += ${LIB_UNITY_DIR}/src
VPATH += ${LIB_UNITY_DIR}/src
SRCS += unity.c
# Add this to PROJ_CFLAGS so that custom config is only used when compiling for
# running on actual hardware.  Otherwise, we are unit testing on the host machine
# and should use the default config
PROJ_CFLAGS += -DUNITY_INCLUDE_CONFIG_H

# Test Options
ifeq "(_OS)" "windows"
$(warning Native Windows environment detected!  Unit testing is currently configured for GCC)
# TODO: MSVC/MinGW support (?)
endif
TEST_CC ?= gcc
ifeq "$(BUILD_DIR)" ""
TEST_BUILD_DIR ?= $(CURDIR)/build/unittest
else
TEST_BUILD_DIR ?= $(BUILD_DIR)/unittest
endif
TEST_SRC_DIR ?= $(CURDIR)/test
TEST_OUTPUT_BINARY ?= $(PROJECT)_unittest

TEST_SRCS += $(LIB_UNITY_DIR)/src/unity.c
TEST_SRCS += $(sort $(wildcard $(TEST_SRC_DIR)/*.c))
TEST_CFLAGS += -I$(LIB_UNITY_DIR)/src
# Add include paths from main project so test code can locate any relevant files
TEST_CFLAGS += ${patsubst %,-I%,$(IPATH)}


$(TEST_BUILD_DIR):
	@echo -  MKDIR $(@)
ifeq "$(_OS)" "windows"
# Make run on native Windows will yield C:/-like paths, but the mkdir commands needs
# paths with backslashes.
	@if not exist ${subst /,\,${@}} mkdir ${subst /,\,${@}}
else
	@mkdir -p ${@}
endif

# Rule for running the test binary.
# Return codes are not ignored so that they can be signaled to the host machine.
.PHONY: test
test: $(TEST_BUILD_DIR)/$(TEST_OUTPUT_BINARY)
	@echo - RUN $(^)
	@ $(^)

# Rule for building the test binary itself.
$(TEST_BUILD_DIR)/$(TEST_OUTPUT_BINARY): $(TEST_SRCS) | $(TEST_BUILD_DIR)
	@echo - CC $(@)
ifeq "$(VERBOSE)" "1"
	$(TEST_CC) $(TEST_CFLAGS) $(TEST_SRCS) -o $(@)
else
	@$(TEST_CC) $(TEST_CFLAGS) $(TEST_SRCS) -o $(@)
endif
