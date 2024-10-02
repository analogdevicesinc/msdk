##############################################################################
 #
 # Copyright 2023-2024 Analog Devices, Inc.
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
# Makefile for linking against the CMSIS-DSP library.

ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../../
endif

ifeq "$(CMSIS_DSP_DIR)" ""
# If PERIPH_DRIVER_DIR is not specified, this Makefile will locate itself.
CMSIS_DSP_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

CMSIS_DSP_VERSION ?= 1.16.2
ifeq ("$(wildcard $(CMSIS_DSP_DIR)/$(CMSIS_DSP_VERSION))","")
$(error Unsupported CMSIS-DSP version '$(CMSIS_DSP_VERSION)'.  Searched $(CMSIS_DSP_DIR))
endif

$(info Enabled CMSIS-DSP v$(CMSIS_DSP_VERSION))

# Include paths...
# DSP files
IPATH += $(CMSIS_DSP_DIR)/$(CMSIS_DSP_VERSION)/Include
# Some newer CMSIS5 core include files, such as cmsis_compiler.h, etc.
IPATH += $(CMSIS_ROOT)/5.9.0/Core/Include

# TODO: Add target check for M3 core micros
# Add processor flag for arm_math.h
PROJ_CFLAGS+=-DARM_MATH_CM4

# Tell core_cm4.h that our CPU has an FPU by defining __FPU_PRESENT
PROJ_CFLAGS+=-D__FPU_PRESENT

# Where to find the DSP library file
PROJ_LDFLAGS += -L$(CMSIS_DSP_DIR)/$(CMSIS_DSP_VERSION)/Lib

ifeq "$(MFLOAT_ABI)" ""
$(warning ***The 'MFLOAT_ABI' Makefile variable is not set!***  Using softfp CMSIS-DSP instructions by default.)
endif

# Link against hard or soft fp
ifeq "$(MFLOAT_ABI)" "hard"
PROJ_LIBS += arm_cortexM4lf_math
else
PROJ_LIBS += arm_cortexM4l_math
endif