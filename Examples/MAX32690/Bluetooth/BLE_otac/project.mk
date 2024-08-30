###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
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
# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
INIT_PERIPHERAL = 0
INIT_CENTRAL = 1

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
TRACE = 1

# **********************************************************
# Firmware builder
# The section below will compile the application specified
# by FW_UPDATE_DIR.  Then, it will combine the application into
# the same single output binary alongside this one. 

# Build directory for image that peer will be updated with
FW_UPDATE_DIR=../BLE_otas

BUILD_DIR := $(abspath ./build)
FW_BUILD_DIR := $(BUILD_DIR)/buildfw

# Firmware update files, do not rename
FW_UPDATE_BIN=$(FW_BUILD_DIR)/fw_update.bin
FW_UPDATE_OBJ=$(FW_BUILD_DIR)/fw_update.o

# This is the 'magic' line that gets the linker to combine in
# the external application's object file.
PROJ_OBJS = ${FW_UPDATE_OBJ}

# Target for creating the fw_update bin file
.PHONY: fw_bin
fw_bin: $(FW_UPDATE_BIN)

${FW_UPDATE_BIN}:
	$(MAKE) -C ${FW_UPDATE_DIR} BUILD_DIR=$(FW_BUILD_DIR) BUILD_BOOTLOADER=0 PROJECT=fw_update
	$(MAKE) -C $(FW_UPDATE_DIR) BUILD_DIR=$(FW_BUILD_DIR) $(FW_UPDATE_BIN)

# Target for creating the firmware update obj file
.PHONY: fw_obj
fw_obj: $(FW_UPDATE_OBJ)

${FW_UPDATE_OBJ}: fw_update.S ${FW_UPDATE_BIN}
	${CC} ${AFLAGS} -o ${@} -c fw_update.S

ifeq ($(BOARD),FTHR)
$(error ERR_NOTSUPPORTED: This project is not supported for the FTHR board)
endif


# set ADVTISEMENT name you want to connect
ADV_NAME?=DATS
PROJ_CFLAGS += -DADV_NAME=\"$(ADV_NAME)\"



### CONFIGURE security
# /*! TRUE to initiate security upon connection*/
PROJ_CFLAGS += -DINIT_SECURITY=TRUE


