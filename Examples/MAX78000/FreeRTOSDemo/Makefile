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
# Don't edit this file!  Edit "project.mk" instead...

# This is the core Makefile for a MaximSDK project.  The configuration below
# is made to be suitable for most use-cases, and you normally shouldn't ever
# need to edit this file directly.  Instead, if you do need to modify the build system
# for your project use the "project.mk" file to override and/or add to the
# various "API"-like configuration variables that are available.

# You can also set most configuration variables on the command-line or in your
# system's environment variables.  If you're using a supported IDE, it will
# offer its own options for overriding the available configuration variables.

# For more details on what configuration variables are available, see the sections 
# of this Makefile below.  Each section contains a description and a list of 
# options.

# For a more comprehensive overview of Make itself, see https://www.gnu.org/software/make/manual/


# *******************************************************************************
# Set the target microcontroller and board to compile for.  

# Every TARGET microcontroller has some Board Support Packages (BSPs) that are available
# for it under the MaximSDK/Libraries/Boards/TARGET folder.  The BSP that gets
# selected is MaximSDK/Libraries/Boards/TARGET/BOARD.

# Configuration Variables:
# - TARGET : Override the default target microcontroller.  Ex: TARGET=MAX78000
# - BOARD : Override the default BSP (case sensitive).  Ex: BOARD=EvKit_V1, BOARD=FTHR_RevA


ifeq "$(TARGET)" ""
# Default target microcontroller
TARGET := MAX78000
TARGET_UC := MAX78000
TARGET_LC := max78000
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
DEPTH=../../../
MAXIM_PATH = $(abspath $(DEPTH))
$(warning Warning:  MAXIM_PATH is not set!  It's recommended to set MAXIM_PATH in your environment or in project.mk)
$(warning Warning:  Attempting to use $(MAXIM_PATH) calculated from relative path)
else
# Sanitize MAXIM_PATH for backslashes
MAXIM_PATH := $(subst \,/,$(MAXIM_PATH))
# Locate some other useful paths...
LIBS_DIR := $(abspath $(MAXIM_PATH)/Libraries)
CMSIS_ROOT := $(LIBS_DIR)/CMSIS
endif

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
VPATH := $(VPATH)

# Where to find header files for this project
IPATH += .
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
LINKERFILE ?= $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC).ld

# This path contains system-level intialization files for the target micro.  Add to the build.
VPATH += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source

# *******************************************************************************
# Libraries

# This section offers "toggle switches" to include or exclude the libraries that
# are available in the MaximSDK.  Set a configuration variable to 1 to include the
# library in the build, or 0 to exclude.

# Each library may also have its own library specific configuration variables.

# Configuration variables:
# - LIB_BOARD : Include the Board-Support Package (BSP) library. (Enabled by default)
# - LIB_PERIPHDRIVERS : Include the peripheral driver library.  (Enabled by default)
# - LIB_CMSIS_DSP : Include the CMSIS-DSP library.
# - LIB_CORDIO : Include the Cordio BLE library
# - LIB_FCL : Include the Free Cryptographic Library (FCL)
# - LIB_FREERTOS : Include the FreeRTOS and FreeRTOS-Plus-CLI libraries
# - LIB_LC3 : Include the Low Complexity Communication Codec (LC3) library
# - LIB_LTTLEFS : Include the "little file system" (littleFS) library
# - LIB_LWIP : Include the lwIP library
# - LIB_MAXUSB : Include the MAXUSB library
# - LIB_SDHC : Include the SDHC library


# BSP (Enabled by default)
# ************************
LIB_BOARD ?= 1
ifeq ($(LIB_BOARD), 1)
BOARD_DIR := $(LIBS_DIR)/Boards/$(TARGET_UC)/$(BOARD)
include $(BOARD_DIR)/board.mk
endif
# ************************

# PeriphDrivers (Enabled by default)
# ************************
LIB_PERIPHDRIVERS ?= 1
ifeq ($(LIB_PERIPHDRIVERS), 1)
PERIPH_DRIVER_DIR := $(LIBS_DIR)/PeriphDrivers
include $(PERIPH_DRIVER_DIR)/periphdriver.mk
endif
# ************************

# CMSIS-DSP (Disabled by default)
# ************************
LIB_CMSIS_DSP ?= 0
ifeq ($(LIB_CMSIS_DSP), 1)
# Include the CMSIS-DSP library
include $(LIBS_DIR)/CMSIS/5.9.0/DSP/CMSIS-DSP.mk
endif
# ************************

# Cordio (Disabled by default)
# ************************
LIB_CORDIO ?= 0
ifeq ($(LIB_CORDIO), 1)
# Cordio Library Options
DEBUG           ?= 1
TRACE           ?= 1
BT_VER          ?= 9
INIT_CENTRAL    ?= 1
INIT_OBSERVER   ?= 1
INIT_ENCRYPTED  ?= 1

# Enter standby mode when idle
STANDBY_ENABLED ?= 0

# Select either option, or both for combined Host and Controller on single core
BLE_HOST        ?= 1
BLE_CONTROLLER  ?= 1

# Disable these trace messages for the speed testing
PROJ_CFLAGS += -DATT_TRACE_ENABLED=0 -DHCI_TRACE_ENABLED=0

# Include the Cordio Library
CORDIO_DIR ?= $(LIBS_DIR)/Cordio
include $(CORDIO_DIR)/platform/targets/maxim/build/cordio.mk
endif
# ************************

# FCL (Disabled by default)
# ************************
LIB_FCL ?= 0
ifeq ($(LIB_FCL), 1)
FCL_DIR  ?= $(LIBS_DIR)/FCL
include $(FCL_DIR)/fcl.mk
endif
# ************************

# FreeRTOS (Disabled by default)
# ************************
LIB_FREERTOS ?= 0
ifeq ($(LIB_FREERTOS), 1)
# Where to find FreeRTOSConfig.h
RTOS_CONFIG_DIR ?= .

# Include FreeRTOS-Plus-CLI
IPATH += $(LIBS_DIR)/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI
VPATH += $(LIBS_DIR)/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI
SRCS += FreeRTOS_CLI.c

# Include the FreeRTOS library
include $(LIBS_DIR)/FreeRTOS/freertos.mk
endif
# ************************

# LC3 (Disabled by default)
# ************************
LIB_LC3 ?= 0
ifeq ($(LIB_LC3), 1)
LC3_ROOT ?= $(LIBS_DIR)/LC3
include $(LC3_ROOT)/build/sources.mk
endif
# ************************

# littleFS (Disabled by default)
# ************************
LIB_LITTLEFS ?= 0
ifeq ($(LIB_LITTLEFS), 1)
LITTLEFS_DIR ?= $(LIBS_DIR)/littlefs
include $(LITTLEFS_DIR)/littlefs.mk
endif
# ************************

# lwIP (Disabled by default)
# ************************
LIB_LWIP ?= 0
ifeq ($(LIB_LWIP), 1)
LWIP_DIR ?= $(LIBS_DIR)/lwIP
include $(LWIP_DIR)/lwip.mk
endif
# ************************

# MAXUSB (Disabled by default)
# ************************
LIB_MAXUSB ?= 0
ifeq ($(LIB_MAXUSB), 1)
MAXUSB_DIR ?= $(LIBS_DIR)/MAXUSB
include $(MAXUSB_DIR)/maxusb.mk
endif
# ************************

# SDHC (Disabled by default)
# ************************
LIB_SDHC ?= 0
ifeq ($(LIB_SDHC), 1)
# Set the SDHC driver directory
SDHC_DRIVER_DIR ?= $(LIBS_DIR)/SDHC

# Set the FAT32 driver directory
FAT32_DRIVER_DIR ?= $(SDHC_DRIVER_DIR)/ff13

# Include the SDHC library
include $(FAT32_DRIVER_DIR)/fat32.mk
include $(SDHC_DRIVER_DIR)/sdhc.mk
endif
# ************************

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
