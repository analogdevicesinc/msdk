################################################################################
 # Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 # 
 # This software is protected by copyright laws of the United States and
 # of foreign countries. This material may also be protected by patent laws
 # and technology transfer regulations of the United States and of foreign
 # countries. This software is furnished under a license agreement and/or a
 # nondisclosure agreement and may only be used or reproduced in accordance
 # with the terms of those agreements. Dissemination of this information to
 # any party or parties not specified in the license agreement and/or
 # nondisclosure agreement is expressly prohibited.
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 ###############################################################################
      
# This is the name of the build output file
ifeq "$(PROJECT)" ""
PROJECT=max32570
endif

# Specify the target processor
ifeq "$(TARGET)" ""
TARGET=MAX32570
endif

# Create Target name variables
TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)

# Select 'GCC' or 'IAR' compiler
COMPILER=GCC

# Specify the board used
ifeq "$(BOARD)" ""
BOARD=QN_EvKit_V1
endif

# This is the path to the CMSIS root directory
ifeq "$(MAXIM_PATH)" ""
LIBS_DIR=../../../Libraries
else
LIBS_DIR=/$(subst \,/,$(subst :,,$(MAXIM_PATH))/Libraries)
endif

CMSIS_ROOT=$(LIBS_DIR)/CMSIS
SECURE_BOOT_TOOLS_DIR=$(LIBS_DIR)/../Tools/SBT
CA_SIGN_BUILD=$(SECURE_BOOT_TOOLS_DIR)/bin/sign_app
BUILD_SESSION=$(SECURE_BOOT_TOOLS_DIR)/bin/build_scp_session
TEST_KEY=$(SECURE_BOOT_TOOLS_DIR)/devices/$(TARGET_UC)/keys/maximtestcrk.key
SCP_PACKETS=scp_packets

#Use this for other library make files so they are all based off the same as root as the project
export CMSIS_ROOT

# Source files for this test (add path to VPATH below)
SRCS  = main.c
SRCS += sla_header.c

# Where to find source files for this test
VPATH=.
VPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source

# Where to find header files for this test
IPATH = .

# Enable assertion checking for development
PROJ_CFLAGS+=-DMXC_ASSERT_ENABLE 

# Enable all warnings
PROJ_CFLAGS+=-Wall

# Specify the target revision to override default
# "A2" in ASCII
# TARGET_REV=0x4132

# Use this variables to specify and alternate tool path
#TOOL_DIR=/opt/gcc-arm-none-eabi-4_8-2013q4/bin


# Use these variables to add project specific tool options
#PROJ_CFLAGS+=--specs=nano.specs
#PROJ_LDFLAGS+=--specs=nano.specs

# Point this variable to a startup file to override the default file
#STARTUPFILE=start.S

# Set MXC_OPTIMIZE to override the default optimization level 
MXC_OPTIMIZE_CFLAGS=-O1

# Point this variable to a linker file to override the default file
LINKER=$(TARGET_LC).ld
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(LINKER)

################################################################################
# Include external library makefiles here

# Include the BSP
BOARD_DIR=$(LIBS_DIR)/Boards/$(TARGET_UC)/$(BOARD)
include $(BOARD_DIR)/board.mk

# Include the peripheral driver
PERIPH_DRIVER_DIR=$(LIBS_DIR)/PeriphDrivers
include $(PERIPH_DRIVER_DIR)/periphdriver.mk
export PERIPH_DRIVER_DIR

################################################################################
# Include the rules for building for this target. All other makefiles should be
# included before this one.
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

all:
ifneq "$(LINKER)" "$(TARGET_LC)_ram.ld"	
	arm-none-eabi-objcopy $(BUILD_DIR)/$(PROJECT).elf -R .sig -O binary $(BUILD_DIR)/$(PROJECT).bin
	@echo " "
	arm-none-eabi-size --format=berkeley $(BUILD_DIR)/$(PROJECT).elf
	@echo " "
	$(CA_SIGN_BUILD) -c $(TARGET_UC) key_file=$(TEST_KEY) algo=ecdsa header=no ca=$(BUILD_DIR)/$(PROJECT).bin sca=$(BUILD_DIR)/$(PROJECT).sbin
	@echo " "
	arm-none-eabi-objcopy  $(BUILD_DIR)/$(PROJECT).elf --update-section .sig=$(BUILD_DIR)/$(PROJECT).sig
endif

libclean: 
	$(MAKE)  -f ${PERIPH_DRIVER_DIR}/periphdriver.mk clean.periph
slaclean: 
	rm -r ${SCP_PACKETS}/

# The rule to clean out all the build products.
distclean: clean libclean slaclean

sla: all
	$(BUILD_SESSION) -c $(TARGET_UC) key_file=$(TEST_KEY) ${SCP_PACKETS} $(BUILD_DIR)/$(PROJECT).sbin
