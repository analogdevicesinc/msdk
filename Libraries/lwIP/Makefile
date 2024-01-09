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

ifeq "$(LWIP_DIR)" ""
# If the location of the library is not specified, this Makefile will locate itself.
LWIP_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
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

include ${LWIP_DIR}/lwip_files.mk

# Add library files to the build.  Since this file drives the
# actual build of the library, we add them to IPATH, SRCS, and VPATH.
IPATH += $(LWIP_INCLUDE_DIR)
SRCS  += $(LWIP_C_FILES)
VPATH += $(dir $(SRCS))

# Open compile guard flag
PROJ_CFLAGS+=-DWITH_LWIP

# Use absolute paths if building within eclipse environment.
ifeq "$(ECLIPSE)" "1"
SRCS := $(abspath $(SRCS))
endif

# Only building libraries.
MAKECMDGOALS=lib

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk
