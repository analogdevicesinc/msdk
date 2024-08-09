# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

BLE_HOST = 0
BLE_CONTROLLER = 1

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
TRACE = 2

BUILD_BOOTLOADER?=1

AUTOSEARCH=0
VPATH += .

SRCS += main.c

USE_INTERNAL_FLASH ?=1
ifeq ($(USE_INTERNAL_FLASH), 1)
PROJ_CFLAGS += -DOTA_INTERNAL=1
LINKERFILE = ota_internal_mem.ld
#SRCS += wdxs_file_int.c
else
LINKERFILE = ota_external_mem.ld
SRCS += wdxs_file_ext.c
endif

# build bootloader
ifeq ($(BUILD_BOOTLOADER), 1)
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
endif

# SET advertising name
ADV_NAME?=DATS
PROJ_CFLAGS += -DADV_NAME=\"$(ADV_NAME)\"


### CONFIGURE security
# /*! TRUE to initiate security upon connection*/
PROJ_CFLAGS += -DINIT_SECURITY=TRUE
