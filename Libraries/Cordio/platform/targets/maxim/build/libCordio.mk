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

# This is the name of the build output file
PROJECT_NAME=cordio

ifeq "$(CORDIO_DIR)" ""
# If CORDIO_DIR is not specified, this Makefile will locate itself.
CORDIO_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../../../..
endif

# Use these to specify the project.
ifeq "$(CORDIO_LIB_VAR)" ""
override PROJECT=$(PROJECT_NAME)
else
override PROJECT=$(PROJECT_NAME)_$(CORDIO_LIB_VAR)
endif

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


ifeq "${LIBS_DIR}" ""
LIBS_DIR := ${CMSIS_ROOT}/../../Libraries
endif

include ${LIBS_DIR}/Cordio/platform/targets/maxim/build/cordio.mk

# Convert Cordio definitions to Maxim CMSIS definitions
PROJ_CFLAGS     += $(addprefix -D,$(sort $(CFG_DEV))) # Remove duplicates
PROJ_AFLAGS     += -DPAL_NVM_SIZE=$(PAL_NVM_SIZE)
SRCS            += $(C_FILES)
VPATH           += %.c $(sort $(dir $(C_FILES)))
IPATH           += $(INC_DIRS)

# Add dependencies in the Board library and the PeripheralDrivers
IPATH += ${LIBS_DIR}/MiscDrivers/PushButton
include ${LIBS_DIR}/PeriphDrivers/periphdriver.mk

# Use absolute paths if building within eclipse environment.
ifeq "$(ECLIPSE)" "1"
SRCS := $(abspath $(SRCS))
endif

# Only building libraries.
MAKECMDGOALS=lib

# Include the rules for building for this target
include ${LIBS_DIR}/CMSIS/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk
