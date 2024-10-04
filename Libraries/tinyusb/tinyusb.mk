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
else
TINYUSB_BUILD_DIR = $(abspath $(BUILD_DIR)/tinyusb)
endif

# Path for tusb_config.h
# (Defaults to root directory of external project folder)
TINYUSB_CONFIG_DIR ?= .
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
SRCS += \
        ${TINYUSB_DIR}/src/tusb.c \
        ${TINYUSB_DIR}/src/common/tusb_fifo.c \
        ${TINYUSB_DIR}/src/device/usbd.c \
        ${TINYUSB_DIR}/src/device/usbd_control.c \
        ${TINYUSB_DIR}/src/class/audio/audio_device.c \
        ${TINYUSB_DIR}/src/class/bth/bth_device.c \
        ${TINYUSB_DIR}/src/class/cdc/cdc_device.c \
        ${TINYUSB_DIR}/src/class/dfu/dfu_device.c \
        ${TINYUSB_DIR}/src/class/dfu/dfu_rt_device.c \
        ${TINYUSB_DIR}/src/class/hid/hid_device.c \
        ${TINYUSB_DIR}/src/class/midi/midi_device.c \
        ${TINYUSB_DIR}/src/class/msc/msc_device.c \
        ${TINYUSB_DIR}/src/class/net/ecm_rndis_device.c \
        ${TINYUSB_DIR}/src/class/net/ncm_device.c \
        ${TINYUSB_DIR}/src/class/usbtmc/usbtmc_device.c \
        ${TINYUSB_DIR}/src/class/video/video_device.c \
        ${TINYUSB_DIR}/src/class/vendor/vendor_device.c \
        ${TINYUSB_DIR}/src/class/cdc/cdc_host.c \
        ${TINYUSB_DIR}/src/class/hid/hid_host.c \
        ${TINYUSB_DIR}/src/class/msc/msc_host.c \
        ${TINYUSB_DIR}/src/class/vendor/vendor_host.c \
        ${TINYUSB_DIR}/src/portable/mentor/musb/dcd_musb.c \
        ${TINYUSB_DIR}/hw/bsp/board.c

endif #TINYUSB_MK

