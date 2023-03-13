################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

# ** Readme! **
# Don't edit this file unless you know what you are doing.

# Specify the project variant.
ifeq "$(MFLOAT_ABI)" "hardfp"
PROJECT_VARIANT=hardfp
else
ifeq "$(MFLOAT_ABI)" "hard"
PROJECT_VARIANT=hardfp
else
PROJECT_VARIANT=softfp
endif
endif

# Use these to specify the project.
ifeq "$(PROJECT_VARIANT)" ""
override PROJECT=$(PROJECT_NAME)
else
override PROJECT=$(PROJECT_NAME)_$(PROJECT_VARIANT)
endif

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC ?= $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC ?= $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

ifeq "$(BUILD_DIR)" ""
BUILD_DIR=./Build
endif

# This is the path to the CMSIS root directory
ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=$(MAXIM_PATH)/Libraries/CMSIS
endif

CUSTOM_LIB_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CUSTOM_LIB_INCLUDE_DIR = $(CUSTOM_LIB_DIR)
CUSTOM_LIB_C_FILES += $(shell find $(CUSTOM_LIB_DIR) -name "*.c")
# # Where to find header files for this project
IPATH += $(CUSTOM_LIB_INCLUDE_DIR)
SRCS  += $(CUSTOM_LIB_C_FILES)
SRCS  += $(CUSTOM_LIB_A_FILES)
VPATH += $(dir $(CUSTOM_LIB_C_FILES))

# Use absolute paths if building within eclipse environment.
ifeq "$(ECLIPSE)" "1"
SRCS := $(abspath $(SRCS))
endif

# Only building libraries.
MAKECMDGOALS=lib

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
# .DEFAULT_GOAL ?= lib
