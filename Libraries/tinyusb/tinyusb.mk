###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

ifeq "$(TINYUSB_DIR)" ""
$(error TINYUSB_DIR must be specified")
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
TINYUSB_BUILD_DIR=$(CURDIR)/build/tinyusb
else
TINYUSB_BUILD_DIR=$(BUILD_DIR)/tinyusb
endif

# Export paths needed by the peripheral driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
TINYUSB_BUILD_DIR := ${abspath ${TINYUSB_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export PERIPH_DRIVER_DIR := ${abspath ${PERIPH_DRIVER_DIR}}

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS
export VERBOSE
export TINYUSB_CONFIG_DIR := ${abspath ${TINYUSB_CONFIG_DIR}}

# Configure the correct TUSB_MCU
ifeq "$(TARGET_UC)" "MAX32650"
TARGET_USB=OPT_MCU_MAX32650
endif
ifeq "$(TARGET_UC)" "MAX32665"
TARGET_USB=OPT_MCU_MAX32666
endif
ifeq "$(TARGET_UC)" "MAX32666"
TARGET_USB=OPT_MCU_MAX32666
endif
ifeq "$(TARGET_UC)" "MAX32690"
TARGET_USB=OPT_MCU_MAX32690
endif
ifeq "$(TARGET_UC)" "MAX78002"
TARGET_USB=OPT_MCU_MAX78002
endif

#Do not enable Host mode support.
PROJ_CFLAGS += -DCFG_TUH_ENABLED=0

#Do not enable Type-C mode
PROJ_CFLAGS += -DCFG_TUC_ENABLED=0

#Enable device mode
PROJ_CFLAGS += -DCFG_TUD_ENABLED=1

#Support high speed mode
PROJ_CFLAGS += -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

#Let TUSB know the MCU
PROJ_CFLAGS += -DCFG_TUSB_MCU=$(TARGET_USB)

IPATH += ${TINYUSB_DIR}/src/
IPATH += ${TINYUSB_DIR}/hw/

# Add to library list
LIBS += ${TINYUSB_BUILD_DIR}/tinyusb.a

# Add rule to build the Driver Library
${TINYUSB_BUILD_DIR}/tinyusb.a: FORCE
	$(MAKE) -C ${TINYUSB_DIR} lib BUILD_DIR=${TINYUSB_BUILD_DIR}

