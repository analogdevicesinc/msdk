###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This
 # software is proprietary to Analog Devices, Inc. and its licensors.
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
override PROJECT=tinyusb

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif
TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))
$(info $(TARGET_UC))

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

# This is the path to the peripheral drivers
ifeq "$(PERIPH_DRIVER_DIR)" ""
PERIPH_DRIVER_DIR=../PeriphDrivers/$(TARGET_UC)
endif

ifeq "$(BUILD_DIR)" ""
BUILD_DIR=./Build
endif

# This is the path to the CMSIS root directory
ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../CMSIS
endif

FREERTOS_DIR_REL := ../FreeRTOS
FREERTOS_DIR := $(abspath $(FREERTOS_DIR_REL))

# Path for tusb_config.h
ifeq "$(TINYUSB_CONFIG_DIR)" ""
TINYUSB_CONFIG_DIR=.
endif

#Do not enable Host mode support. Only device
PROJ_CFLAGS += -DCFG_TUH_ENABLED=0

#Do not enable Type-C mode.
PROJ_CFLAGS += -DCFG_TUC_ENABLED=0

#Enable device mode
PROJ_CFLAGS += -DCFG_TUD_ENABLED=1

#Support high speed mode
PROJ_CFLAGS += -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

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

#Let TUSB know the MCU
PROJ_CFLAGS += -DCFG_TUSB_MCU=$(TARGET_USB)

# C source files
SRCS += \
	src/tusb.c \
	src/common/tusb_fifo.c \
	src/device/usbd.c \
	src/device/usbd_control.c \
	src/class/audio/audio_device.c \
	src/class/bth/bth_device.c \
	src/class/cdc/cdc_device.c \
	src/class/dfu/dfu_device.c \
	src/class/dfu/dfu_rt_device.c \
	src/class/hid/hid_device.c \
	src/class/midi/midi_device.c \
	src/class/msc/msc_device.c \
	src/class/net/ecm_rndis_device.c \
	src/class/net/ncm_device.c \
	src/class/usbtmc/usbtmc_device.c \
	src/class/video/video_device.c \
	src/class/vendor/vendor_device.c \
	src/class/cdc/cdc_host.c \
	src/class/hid/hid_host.c \
	src/class/msc/msc_host.c \
	src/class/vendor/vendor_host.c \
	src/portable/mentor/musb/dcd_musb.c \
	hw/bsp/board.c

IPATH += src/
IPATH += hw/
IPATH += $(TINYUSB_CONFIG_DIR)
IPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET)/Include
IPATH += $(CMSIS_ROOT)/5.9.0/Core/Include
IPATH += $(PERIPH_DRIVER_DIR)/Include/$(TARGET_UC)
IPATH += ${FREERTOS_DIR}/Source/portable/$(COMPILER)/ARM_CM4F
IPATH += ${FREERTOS_DIR}/Source/include
VPATH += ./

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL ?= lib
