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

ifeq "$(BOARD_DIR)" ""
# This Makefile will self-locate if BOARD_DIR is not specified.
BOARD_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

# Source files for this test (add path to VPATH below)
SRCS += board.c
SRCS += stdio.c
SRCS += led.c
SRCS += pb.c
SRCS += tft_ili9341.c
SRCS += camera.c
SRCS += ov7692.c
SRCS += sccb.c
SRCS += max20303.c
SRCS += max9867.c
ifeq "$(RISCV_CORE)" ""
SRCS += N01S830HA.c
SRCS += fastspi.c
else
$(warning Warning: Skipping N01S830HA drivers for RISC-V core)
endif

PROJ_CFLAGS+=-DCAMERA_OV7692

MISC_DRIVERS_DIR ?= $(MAXIM_PATH)/Libraries/MiscDrivers

# Where to find BSP source files
VPATH += $(BOARD_DIR)/Source
VPATH += $(MISC_DRIVERS_DIR)
VPATH += $(MISC_DRIVERS_DIR)/Camera
VPATH += $(MISC_DRIVERS_DIR)/Display
VPATH += $(MISC_DRIVERS_DIR)/LED
VPATH += $(MISC_DRIVERS_DIR)/PushButton
VPATH += $(MISC_DRIVERS_DIR)/PMIC
VPATH += $(MISC_DRIVERS_DIR)/Touchscreen
VPATH += $(MISC_DRIVERS_DIR)/CODEC
VPATH += $(MISC_DRIVERS_DIR)/SRAM

# Where to find BSP header files
IPATH += $(BOARD_DIR)/Include
IPATH += $(MISC_DRIVERS_DIR)
IPATH += $(MISC_DRIVERS_DIR)/Camera
IPATH += $(MISC_DRIVERS_DIR)/Display
IPATH += $(MISC_DRIVERS_DIR)/LED
IPATH += $(MISC_DRIVERS_DIR)/PushButton
IPATH += $(MISC_DRIVERS_DIR)/PMIC
IPATH += $(MISC_DRIVERS_DIR)/Touchscreen
IPATH += $(MISC_DRIVERS_DIR)/CODEC
IPATH += $(MISC_DRIVERS_DIR)/SRAM

include $(MISC_DRIVERS_DIR)/Display/fonts/fonts.mk
