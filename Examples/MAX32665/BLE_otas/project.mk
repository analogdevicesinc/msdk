# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# If you have secure version of MCU (MAX32666), set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Enable CORDIO library
LIB_CORDIO = 1

# Cordio library options
STANDBY_ENABLED = 0
INIT_PERIPHERAL = 1
INIT_BROADCASTER = 1
INIT_CENTRAL = 0
INIT_OBSERVER = 0


# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

AUTOSEARCH=0
VPATH += .
SRCS += stack_dats.c 
SRCS += dats_main.c
SRCS += main.c

PROJ_CFLAGS += -Wl,--section-start=.text=0x10004000

USE_INTERNAL_FLASH ?=0
ifeq ($(USE_INTERNAL_FLASH), 1)
LINKERFILE = ota_internal_mem.ld
SRCS += wdxs_file_int.c
else
LINKERFILE = ota_external_mem.ld
SRCS += wdxs_file_ext.c
endif

# build bootloader
BOOTLOADER_DIR=../Bootloader

BUILD_DIR := $(abspath ./build)
BOOTLOADER_BUILD_DIR := $(BUILD_DIR)/buildbl

BOOTLOADER_BIN=$(BOOTLOADER_BUILD_DIR)/bootloader.bin
BOOTLOADER_OBJ=$(BOOTLOADER_BUILD_DIR)/bootloader.o

PROJ_OBJS = ${BOOTLOADER_OBJ}

.PHONY: bl_bin
bl_bin: $(BOOTLOADER_BIN)

${BOOTLOADER_BIN}:
	$(MAKE) -C ${BOOTLOADER_DIR} BUILD_DIR=$(BOOTLOADER_BUILD_DIR) PROJECT=bootloader
	$(MAKE) -C $(BOOTLOADER_DIR) BUILD_DIR=$(BOOTLOADER_BUILD_DIR) $(BOOTLOADER_BIN)

.PHONY: bl_obj
bl_obj: $(BOOTLOADER_OBJ)

${BOOTLOADER_OBJ}: bl_build.S ${BOOTLOADER_BIN}
	${CC} ${AFLAGS} -o ${@} -c bl_build.S 
