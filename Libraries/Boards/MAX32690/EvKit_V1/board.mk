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

ifeq "$(BOARD_DIR)" ""
$(error BOARD_DIR must be set)
endif

# Source files for this test (add path to VPATH below)
SRCS += board.c
SRCS += stdio.c
SRCS += led.c
SRCS += mx25.c
SRCS += pb.c
SRCS += tft_st7735.c
SRCS += max9867.c

PROJ_CFLAGS+=-DEXT_FLASH_MX25
# Define "MAX32690GTK_PACKAGE_TQFN", which some drivers use to work around MAX32690 Rev A2 Errata #16:
# https://www.analog.com/media/en/technical-documentation/data-sheets/max32690_a2_errata_rev2.pdf
PROJ_CFLAGS += -DMAX32690GTK_PACKAGE_TQFN

MISC_DRIVERS_DIR ?= $(MAXIM_PATH)/Libraries/MiscDrivers

# Where to find BSP source files
VPATH += $(BOARD_DIR)/Source
VPATH += $(MISC_DRIVERS_DIR)
VPATH += $(MISC_DRIVERS_DIR)/LED
VPATH += $(MISC_DRIVERS_DIR)/PushButton
VPATH += $(MISC_DRIVERS_DIR)/ExtMemory
VPATH += $(MISC_DRIVERS_DIR)/Display
VPATH += $(MISC_DRIVERS_DIR)/CODEC

# Where to find BSP header files
IPATH += $(BOARD_DIR)/Include
IPATH += $(MISC_DRIVERS_DIR)
IPATH += $(MISC_DRIVERS_DIR)/LED
IPATH += $(MISC_DRIVERS_DIR)/PushButton
IPATH += $(MISC_DRIVERS_DIR)/ExtMemory
IPATH += $(MISC_DRIVERS_DIR)/Display
IPATH += $(MISC_DRIVERS_DIR)/CODEC

include $(MISC_DRIVERS_DIR)/Display/fonts/fonts.mk
