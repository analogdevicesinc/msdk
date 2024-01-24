###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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

ifeq "$(MAXUSB_DIR)" ""
$(error MAXUSB_DIR must be specified")
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
MAXUSB_BUILD_DIR=$(CURDIR)/build/MAXUSB
else
MAXUSB_BUILD_DIR=$(BUILD_DIR)/MAXUSB
endif

# Export paths needed by the peripheral driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
MAXUSB_BUILD_DIR := ${abspath ${MAXUSB_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export PERIPH_DRIVER_DIR := ${abspath ${PERIPH_DRIVER_DIR}}

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS

# Add to library list
LIBS += ${MAXUSB_BUILD_DIR}/maxusb.a

# Select Full Speed or High Speed Library
ifeq "$(TARGET_UC)" "MAX32572"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32650"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32665"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32666"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32667"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32668"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32655"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32656"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32570"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX32690"
TARGET_USB=MUSBHSFC
endif
ifeq "$(TARGET_UC)" "MAX78002"
TARGET_USB=MUSBHSFC
endif

# Add to include directory list
ifeq "$(TARGET_USB)" "MUSBHSFC"
IPATH += ${MAXUSB_DIR}/include/core/musbhsfc
else
IPATH += ${MAXUSB_DIR}/include/core/arm
endif
IPATH += ${MAXUSB_DIR}/include
IPATH += ${MAXUSB_DIR}/include/core
IPATH += ${MAXUSB_DIR}/include/enumerate
IPATH += ${MAXUSB_DIR}/include/devclass
IPATH += ${MAXUSB_DIR}/include/dbg_log

# Add rule to build the Driver Library
${MAXUSB_BUILD_DIR}/maxusb.a: FORCE
	$(MAKE) -C ${MAXUSB_DIR} lib BUILD_DIR=${MAXUSB_BUILD_DIR}
