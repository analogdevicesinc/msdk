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

ifeq "$(TINYUSB_MK)" ""
TINYUSB_MK := 1

ifeq "$(TINYUSB_DIR)" ""
# If TINYUSB_DIR is not specified, this Makefile will locate itself.
TINYUSB_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif
export TINYUSB_DIR

# Build in the library folder by default.  Otherwise, build in a tinyusb subdirectory
# of the specified BUILD_DIR 
ifeq "$(BUILD_DIR)" ""
TINYUSB_BUILD_DIR = $(abspath $(TINYUSB_DIR)/build/$(TARGET_UC))
BUILD_DIR := $(TINYUSB_BUILD_DIR)
else
TINYUSB_BUILD_DIR = $(abspath $(BUILD_DIR)/tinyusb)
endif

# Path for tusb_config.h
TINYUSB_CONFIG_DIR ?= $(TINYUSB_DIR)/src/portable/mentor/musb/default_configs
export TINYUSB_CONFIG_DIR := ${abspath ${TINYUSB_CONFIG_DIR}}
IPATH += $(TINYUSB_CONFIG_DIR)

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
${TINYUSB_BUILD_DIR}/tinyusb.a: ${TINYUSB_CONFIG_DIR}/tusb_config.h
	$(MAKE) -C ${TINYUSB_DIR} lib BUILD_DIR=${TINYUSB_BUILD_DIR}

.PHONY: clean.tinyusb
clean.tinyusb:
	$(MAKE) -C ${TINYUSB_DIR} clean

endif #TINYUSB_MK

