# /*******************************************************************************
# * Copyright (C) 2021 Maxim Integrated Products, Inc., All Rights Reserved.
# *
# * Permission is hereby granted, free of charge, to any person obtaining a
# * copy of this software and associated documentation files (the "Software"),
# * to deal in the Software without restriction, including without limitation
# * the rights to use, copy, modify, merge, publish, distribute, sublicense,
# * and/or sell copies of the Software, and to permit persons to whom the
# * Software is furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included
# * in all copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
# * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# * OTHER DEALINGS IN THE SOFTWARE.
# *
# * Except as contained in this notice, the name of Maxim Integrated
# * Products, Inc. shall not be used except as stated in the Maxim Integrated
# * Products, Inc. Branding Policy.
# *
# * The mere transfer of this software does not imply any licenses
# * of trade secrets, proprietary technology, copyrights, patents,
# * trademarks, maskwork rights, or any other form of intellectual
# * property whatsoever. Maxim Integrated Products, Inc. retains all
# * ownership rights.
# *******************************************************************************

# ** Readme! **
# Don't edit this file!  Override settings in "project.mk" instead...

# This is the core Makefile for a MaximSDK project.
# You normally shouldn't need to edit this file.  
# Instead, we offer various configuration variables that
# can be overridden in "project.mk", on the command-line
# or in your system's environment variables.

# See https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration
# for more detailed instructions on how to use this system.

# The instructions above are a lot easier to read than this file, but
# the comments below also outline the available configuration variables
# if you're' familiar with Make.  This file is organized into sub-sections,
# some of which expose config variables.

# For a more comprehensive overview of Make itself, see https://www.gnu.org/software/make/manual/


# *******************************************************************************
# Set the target microcontroller and board to compile for.  

# Every TARGET microcontroller has some Board Support Packages (BSPs) that are 
# available for it under the MaximSDK/Libraries/Boards/TARGET folder.  The BSP 
# that gets selected is MaximSDK/Libraries/Boards/TARGET/BOARD.

# Configuration Variables:
# - TARGET : Override the default target microcontroller.  Ex: TARGET=MAX78000
# - BOARD : Override the default BSP (case sensitive).  Ex: BOARD=EvKit_V1, BOARD=FTHR_RevA


ifeq "$(TARGET)" ""
# Default target microcontroller
TARGET := MAX78002
TARGET_UC := MAX78002
TARGET_LC := max78002
else
# "TARGET" has been overridden in the environment or on the command-line.
# We need to calculate an upper and lowercase version of the part number,
# because paths on Linux and MacOS are case-sensitive.
TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))
endif

# Default board.
BOARD ?= EvKit_V1

# *******************************************************************************
# Locate the MaximSDK

# This Makefile needs to know where to find the MaximSDK, and the MAXIM_PATH variable 
# should point to the root directory of the MaximSDK installation.  Setting this manually
# is usually only required if you're working on the command-line.

# If MAXIM_PATH is not specified, we assume the project still lives inside of the MaximSDK 
# and move up from this project's original location.

# Configuration Variables:
# - MAXIM_PATH : Tell this Makefile where to find the MaximSDK.  Ex:  MAXIM_PATH=C:/MaximSDK


ifneq "$(MAXIM_PATH)" ""
# Sanitize MAXIM_PATH for backslashes
MAXIM_PATH := $(subst \,/,$(MAXIM_PATH))
# Locate some other useful paths...
LIBS_DIR := $(abspath $(MAXIM_PATH)/Libraries)
CMSIS_ROOT := $(LIBS_DIR)/CMSIS
endif

# *******************************************************************************
# Include project Makefile.  We do this after formulating TARGET, BOARD, and MAXIM_PATH 
# in case project.mk needs to reference those values.  However, we also include
# this as early as possible in the Makefile so that it can append to or override
# the variables below.


include ./project.mk
$(info Loaded project.mk)

# *******************************************************************************
# Final path sanitization and re-calculation.  No options here.

ifeq "$(MAXIM_PATH)" ""
# MAXIM_PATH is still not defined...
DEPTH := ../../../
MAXIM_PATH := $(abspath $(DEPTH))
$(warning Warning:  MAXIM_PATH is not set!  It's recommended to set MAXIM_PATH in your environment or in project.mk)
$(warning Warning:  Attempting to use $(MAXIM_PATH) calculated from relative path)
else
# Sanitize MAXIM_PATH for backslashes
MAXIM_PATH := $(subst \,/,$(MAXIM_PATH))
endif

# Final recalculation of LIBS_DIR/CMSIS_ROOT
LIBS_DIR := $(abspath $(MAXIM_PATH)/Libraries)
CMSIS_ROOT := $(LIBS_DIR)/CMSIS

# One final UC/LC check in case user set TARGET in project.mk
TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))

export TARGET
export TARGET_UC
export TARGET_LC
export CMSIS_ROOT
# TODO: Remove dependency on exports for these variables.

# *******************************************************************************
# Set up search paths, and auto-detect all source code on those paths.

# The following paths are searched by default, where "./" is the project directory.
# ./
# ├─ *.h
# ├─ *.c
# ├─include (optional)
#   └─ *.h
# ├─src (optional)
#   └─ *.c

# Configuration Variables:
# - VPATH : Tell this Makefile to search additional locations for source (.c) files.
# 			You should use the "+=" operator with this option.  
#			Ex:  VPATH += your/new/path
# - IPATH : Tell this Makefile to search additional locations for header (.h) files.
# 			You should use the "+=" operator with this option.  
#			Ex:  VPATH += your/new/path
# - SRCS : Tell this Makefile to explicitly add a source (.c) file to the build.
# 			This is really only useful if you want to add a source file that isn't
#			on any VPATH, in which case you can add the full path to the file here.
#			You should use the "+=" operator with this option.
#			Ex:  SRCS += your/specific/source/file.c
# - AUTOSEARCH : Set whether this Makefile should automatically detect .c files on
#				VPATH and add them to the build.  This is enabled by default.  Set
#				to 0 to disable.  If autosearch is disabled, source files must be
#				manually added to SRCS.
#				Ex:  AUTOSEARCH = 0


# Where to find source files for this project.
VPATH += .
VPATH += src
VPATH := $(VPATH)

# Where to find header files for this project
IPATH += .
IPATH += include
IPATH := $(IPATH)

AUTOSEARCH ?= 1
ifeq ($(AUTOSEARCH), 1)
# Auto-detect all C source files on VPATH
SRCS += $(wildcard $(addsuffix /*.c, $(VPATH)))
endif

# Collapse SRCS before passing them on to the next stage
SRCS := $(SRCS)

# *******************************************************************************
# Set the output filename

# Configuration Variables:
# - PROJECT : Override the default output filename.  Ex: PROJECT=MyProject


# The default value creates a file named after the target micro.  Ex: MAX78000.elf
PROJECT ?= $(TARGET_UC)

# *******************************************************************************
# Compiler options

# Configuration Variables:
# - MXC_OPTIMIZE_CFLAGS : Override the default compiler optimization level.  
#			Ex: MXC_OPTIMIZE_CFLAGS = -O2
# - PROJ_CFLAGS : Add additional compiler flags to the build.
#			You should use the "+=" operator with this option. 
#			Ex:  PROJ_CFLAGS += -Wextra
# - MFLOAT_ABI : Set the floating point acceleration level.
#			The only options are "hard", "soft", or "softfp".
#			Ex: MFLOAT_ABI = hard
# - LINKERFILE : Override the default linkerfile.
#			Ex: LINKERFILE = customlinkerfile.ld


# Select 'GCC' or 'IAR' compiler
ifeq "$(COMPILER)" ""
COMPILER := GCC
endif

# Set compiler optimization level
ifeq "$(MAKECMDGOALS)" "release"
# Optimization level for "release" builds
MXC_OPTIMIZE_CFLAGS := -O2
else
# Optimization level for default builds
MXC_OPTIMIZE_CFLAGS := -Og
endif

# Set compiler flags
PROJ_CFLAGS += -Wall # Enable warnings
PROJ_CFLAGS += -DMXC_ASSERT_ENABLE

# Set hardware floating point acceleration.
# Options are:
# - hard
# - soft
# - softfp (default if MFLOAT_ABI is not set)
MFLOAT_ABI ?= softfp
# MFLOAT_ABI must be exported to other Makefiles, who check this too
export MFLOAT_ABI

# Set the default linkerfile.  Since the core Makefiles later add the location of Maxim's
# linkerfiles to VPATH, and the local project directory has also been added to VPATH, Make
# will search both locations for the specified linkerfile if it can't find it by its path alone.
# The result is that overriding LINKERFILE with the filename of one of Maxim's alternate linkerfiles
# (ex: LINKERFILE=max78000_arm.ld) will work just the same as LINKERFILE=mycustom.ld 
# even if mycustom.ld lives locally to this project.

ifeq "$(RISCV_CORE)" ""
# Default linkerfile is only specified for standard Arm-core projects.
# Otherwise, gcc_riscv.mk sets the appropriate riscv linkerfile.
LINKER = $(TARGET_LC).ld
LINKERFILE ?= $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(LINKER)
endif

# This path contains system-level intialization files for the target micro.  Add to the build.
VPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source

# *******************************************************************************
# Libraries

# This section offers "toggle switches" to include or exclude the libraries that
# are available in the MaximSDK.  Set a configuration variable to 1 to include the
# library in the build, or 0 to exclude.

# Each library may also have its own library specific configuration variables.  See
# Libraries/libs.mk for more details.

# Configuration variables:
# - LIB_BOARD : Include the Board-Support Package (BSP) library. (Enabled by default)
# - LIB_PERIPHDRIVERS : Include the peripheral driver library.  (Enabled by default)
# - LIB_CMSIS_DSP : Include the CMSIS-DSP library.
# - LIB_CORDIO : Include the Cordio BLE library
# - LIB_FCL : Include the Free Cryptographic Library (FCL)
# - LIB_FREERTOS : Include the FreeRTOS and FreeRTOS-Plus-CLI libraries
# - LIB_LC3 : Include the Low Complexity Communication Codec (LC3) library
# - LIB_LITTLEFS : Include the "little file system" (littleFS) library
# - LIB_LWIP : Include the lwIP library
# - LIB_MAXUSB : Include the MAXUSB library
# - LIB_SDHC : Include the SDHC library

include $(LIBS_DIR)/libs.mk

# *******************************************************************************
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
