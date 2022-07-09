################################################################################
 # Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
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
 #
 ###############################################################################

# This is the name of the build output file
ifeq "$(PROJECT)" ""
PROJECT=max32572
endif

# Specify the target processor
ifeq "$(TARGET)" ""
TARGET=MAX32572
endif

# Create Target name variables
TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)

# Select 'GCC' or 'IAR' compiler
COMPILER=GCC

# Specify the board used
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
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

ifeq ("$(wildcard $(LIBS_DIR)/NFC)","")
$(error This demo example requires NFC package, Please install NFC package.)
endif

ifeq ("$(wildcard $(LIBS_DIR)/EMV)","")
$(error This demo example requires EMV package, Please install EMV package.)
endif

# Source files for this test (add path to VPATH below)
SRCS  = main.c
SRCS += sla_header.c
SRCS += resources/all_imgs.c
SRCS += src/wrapper_tft.c
SRCS += src/utils.c
# states
SRCS += src/state.c
SRCS += src/state_home.c
SRCS += src/state_idle.c
SRCS += src/state_slide.c
SRCS += src/state_keypad.c
SRCS += src/state_info.c
SRCS += src/state_nfc.c
SRCS += src/state_msr.c
SRCS += src/state_smartcard.c
# tasks
SRCS += src/task_touchscreen.c
SRCS += src/task_keypad.c
SRCS += src/task_animation.c
SRCS += src/task_smartcard.c
SRCS += src/task_nfc.c
SRCS += src/task_msr.c
# smartcard
SRCS += src/ST8034.c
# msr
SRCS += src/msr/msr_sdma.c
# nfc
SRCS += src/nfc/mml_nfc_port.c
SRCS += src/nfc/logging.c
SRCS += src/nfc/emv_l1_stack/iso14443_3b_flow.c
SRCS += src/nfc/emv_l1_stack/iso14443_3a_cmd.c
SRCS += src/nfc/emv_l1_stack/iso14443_3a_flow.c
SRCS += src/nfc/emv_l1_stack/iso14443_3_common.c
SRCS += src/nfc/emv_l1_stack/iso14443_4_transitive.c
SRCS += src/nfc/emv_l1_stack/iso14443_3b_cmd.c
SRCS += src/nfc/contactless_l1_app/EMV_polling_and_loopback.c


# Where to find source files for this test
VPATH=.
VPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source
VPATH += resources
VPATH += src
VPATH += ./src/msr
VPATH += ./src/nfc
VPATH += ./src/nfc/emv_l1_stack
VPATH += ./src/nfc/contactless_l1_app

# Where to find header files for this test
IPATH = .
IPATH += include
IPATH += resources
IPATH += include/nfc
IPATH += include/nfc/contactless_l1_app
IPATH += include/nfc/emv_l1_stack
IPATH += $(LIBS_DIR)/NFC/lib_nfc_pcd_rf_driver_MAX32572/include/nfc
IPATH += $(LIBS_DIR)/NFC/lib_nfc_pcd_pbm/include

# Enable assertion checking for development
PROJ_CFLAGS+=-DMXC_ASSERT_ENABLE

PROJ_AFLAGS+=-D__MSR_SDMA_SIZE=0xA000

# Enable all warnings
PROJ_CFLAGS+=-Wall

# Specify the target revision to override default
# "A2" in ASCII
# TARGET_REV=0x4132

# Use this variables to specify and alternate tool path
#TOOL_DIR=/opt/gcc-arm-none-eabi-4_8-2013q4/bin

PROJ_LIBS += nfc_pcd_pbm_softfp
PROJ_LIBS += nfc_pcd_rf_driver_MAX32572_softfp

# Use these variables to add project specific tool options
PROJ_CFLAGS += -D__$(TARGET_UC)
PROJ_CFLAGS += -D$(BOARD)
#PROJ_CFLAGS+=--specs=nano.specs
#PROJ_LDFLAGS+=--specs=nano.specs
PROJ_LDFLAGS += -L$(LIBS_DIR)/NFC/lib_nfc_pcd_pbm
PROJ_LDFLAGS += -L$(LIBS_DIR)/NFC/lib_nfc_pcd_rf_driver_MAX32572

# Point this variable to a startup file to override the default file
#STARTUPFILE=start.S

# Set MXC_OPTIMIZE to override the default optimization level 
#MXC_OPTIMIZE_CFLAGS=-O0

# Point this variable to a linker file to override the default file
LINKER=$(TARGET_LC).ld
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(LINKER)

# Convert bitmap and font files to all_imgs.c file
$(shell $(LIBS_DIR)/../Tools/BitmapConverter/maxim_bitmap_converter ./resources)

################################################################################
# Include external library makefiles here

# Include the BSP
BOARD_DIR=$(LIBS_DIR)/Boards/$(TARGET_UC)/$(BOARD)
include $(BOARD_DIR)/board.mk

# Include the peripheral driver
PERIPH_DRIVER_DIR=$(LIBS_DIR)/PeriphDrivers
include $(PERIPH_DRIVER_DIR)/periphdriver.mk
export PERIPH_DRIVER_DIR

RTOS_CONFIG_DIR=.
RTOS_DIR=$(LIBS_DIR)/FreeRTOS
include $(RTOS_DIR)/freertos.mk

EMV_DIR=$(LIBS_DIR)/EMV
include $(EMV_DIR)/emv.mk

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
