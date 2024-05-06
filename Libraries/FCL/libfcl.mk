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

# This is the name of the build output file
PROJECT_NAME=libfcl

# Specify the project variant.
ifeq "$(MFLOAT_ABI)" "hardfp"
PROJECT_VARIANT=hardfp
else
ifeq "$(MFLOAT_ABI)" "hard"
PROJECT_VARIANT=hardfp
else
PROJECT_VARIANT=softfp
endif
endif

# Use these to specify the project.
ifeq "$(PROJECT_VARIANT)" ""
PROJECT=$(PROJECT_NAME)
else
PROJECT=$(PROJECT_NAME)_$(PROJECT_VARIANT)
endif

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))
$(info $(TARGET_UC))


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

include ${CMSIS_ROOT}/../FCL/fcl_files.mk

# # Where to find header files for this project
IPATH += $(FCL_INCLUDE_DIR)
SRCS  += $(FCL_C_FILES)
VPATH += $(dir $(SRCS))

# Use absolute paths if building within eclipse environment.
ifeq "$(ECLIPSE)" "1"
SRCS := $(abspath $(SRCS))
endif

# Only building libraries.
MAKECMDGOALS=lib

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk
