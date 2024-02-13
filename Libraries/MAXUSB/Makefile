###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023-2024Analog Devices, Inc. All Rights Reserved. This software
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

# This is the name of the build output file
override PROJECT=maxusb

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



# Source files for this test (add path to VPATH below)
SRCS  = usb.c
SRCS += usb_event.c
SRCS += enumerate.c
SRCS += hid_kbd.c
SRCS += hid_raw.c
SRCS += cdc_acm.c
SRCS += fifo.c
SRCS += msc.c
SRCS += ccid.c
# Generic debug log, useful for minimally-invasive tracing of the USB stack
#SRCS += dbg_log.c

# Where to find source files for this project
ifeq "$(TARGET_USB)" "MUSBHSFC"
VPATH  = src/core/musbhsfc
else
VPATH  = src/core/arm
endif
VPATH += src/core
VPATH += src/devclass
VPATH += src/enumerate
VPATH += src/dbg_log

# Where to find header files for this project
ifeq "$(TARGET_USB)" "MUSBHSFC"
IPATH  = include/core/musbhsfc
else
IPATH  = include/core/arm
endif
IPATH += include/core
IPATH += include/devclass
IPATH += include/enumerate
IPATH += include/dbg_log
IPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET)/Include
IPATH += $(CMSIS_ROOT)/Include
IPATH += $(PERIPH_DRIVER_DIR)/Include/$(TARGET_UC)

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL ?= lib
