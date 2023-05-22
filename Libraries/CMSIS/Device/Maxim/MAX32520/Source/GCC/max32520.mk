################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

ifeq "$(CMSIS_ROOT)" ""
$(error CMSIS_ROOT must be specified)
endif

TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)

# The build directory
ifeq "$(BUILD_DIR)" ""
BUILD_DIR=$(CURDIR)/build
endif

ifeq "$(STARTUPFILE)" ""
STARTUPFILE=startup_$(TARGET_LC).S
endif

ifeq "$(RISCV_CORE)" "" # RISCV
# Default linkerfile is only specified for standard Arm-core projects.
# Otherwise, gcc_riscv.mk sets the appropriate riscv linkerfile.
LINKERFILE ?= $(TARGET_LC).ld
LINKERPATH ?= $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC

ifeq ("$(wildcard $(LINKERFILE))","") # Check if linkerfile exists
# Doesn't exist...

ifneq ("$(wildcard $(LINKERPATH)/$(LINKERFILE))","") # Search GCC folder
$(info Auto-located linkerfile: $(LINKERPATH)/$(LINKERFILE))
# Form full path to linkerfile.  Works around MSYS2 edge case from (see MSDK-903).
LINKERFILE := $(LINKERPATH)/$(LINKERFILE)
else
$(warning Failed to locate linkerfile: $(LINKERFILE))
endif # End search GCC folder

endif # End check if linkerfile exists

endif # End RISCV

ifeq "$(ENTRY)" ""
ENTRY=Reset_Handler
endif

# Default TARGET_REVISION
# "A1" in ASCII
ifeq "$(TARGET_REV)" ""
TARGET_REV=0x4131
endif

# Add target specific CMSIS source files
ifneq (${MAKECMDGOALS},lib)
ifneq (${MAKECMDGOALS},scpa)
SRCS += ${STARTUPFILE}
endif
SRCS += heap.c
SRCS += system_$(TARGET_LC).c
endif

# Add target specific CMSIS source directories
VPATH+=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC
VPATH+=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source

# Add target specific CMSIS include directories
IPATH+=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Include

# Add CMSIS Core files
CMSIS_VER ?= 5.9.0
IPATH+=$(CMSIS_ROOT)/$(CMSIS_VER)/Core/Include

# Add directory with linker include file
LIBPATH+=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC

# Include the rules and goals for building
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/gcc.mk

# Include rules for flashing
include $(CMSIS_ROOT)/../../Tools/Flash/flash.mk
