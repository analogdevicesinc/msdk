################################################################################
 # Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ###############################################################################
# Main configuration
# Below are the most commonly needed configuration options for this Makefile.
# You shouldn't need to touch anything else unless advanced configuration is required.

# Source files to compile for this project (add path to VPATH below)
SRCS += arm_linear_interp_example_f32.c
SRCS += arm_linear_interp_data.c
SRCS += math_helper.c

# Where to find source files for this project
VPATH += .

# Where to find header files for this project
IPATH += .

# Set compiler flags
PROJ_CFLAGS+=-Wall # Enable warnings
PROJ_CFLAGS+=-DMXC_ASSERT_ENABLE # Enable assertion checking for development

# Enable printfs in the example project
PROJ_CFLAGS+=-DSEMIHOSTING

# Set compiler optimization level 
MXC_OPTIMIZE_CFLAGS=-Og

# Set hardware floating point acceleration.
# Options are:
# - hard
# - soft
# - softfp (default if MFLOAT_ABI is not set)
MFLOAT_ABI=hard
export MFLOAT_ABI

 ###############################################################################

# Set the MAKE variable.  This eliminates issues with some setups
MAKE=make

ifeq "$(TARGET)" ""
TARGET=MAX32670
endif

# Default board used.  This is overridden in settings.json
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
endif

# Set output filename
ifeq "$(PROJECT)" ""
PROJECT=$(TARGET)
endif

# Create Target name variables
TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)

# Select 'GCC' or 'IAR' compiler
COMPILER=GCC

# This is the path to the CMSIS root directory
ifeq "$(MAXIM_PATH)" ""
LIBS_DIR=../../../../Libraries
else
LIBS_DIR=/$(subst \,/,$(subst :,,$(MAXIM_PATH))/Libraries)
endif
#LIBS_DIR = ./Libraries

CMSIS_ROOT=$(LIBS_DIR)/CMSIS

#Use this for other library make files so they are all based off the same as root as the project
export CMSIS_ROOT
VPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source

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

# Include the CMSIS-DSP library
include $(CMSIS_ROOT)/5.9.0/DSP/CMSIS-DSP.mk

################################################################################
# Include the rules for building for this target. All other makefiles should be
# included before this one.
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

all:
# 	Extend the functionality of the "all" recipe here

libclean: 
	$(MAKE)  -f ${PERIPH_DRIVER_DIR}/periphdriver.mk clean.periph
	
clean: 
#	Extend the functionality of the "clean" recipe here

# The rule to clean out all the build products.
distclean: clean libclean
