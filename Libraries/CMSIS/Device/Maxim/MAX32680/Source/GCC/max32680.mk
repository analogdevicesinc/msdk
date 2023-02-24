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
ifeq "$(RISCV_CORE)" ""
BUILD_DIR=$(CURDIR)/build
else
BUILD_DIR=$(CURDIR)/buildrv
endif
endif

ifeq "$(STARTUPFILE)" ""
ifeq "$(RISCV_CORE)" ""
STARTUPFILE=startup_$(TARGET_LC).S
else
STARTUPFILE=startup_riscv_$(TARGET_LC).S
endif
endif

ifeq "$(LINKERFILE)" ""
ifeq "$(RISCV_CORE)" ""
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC).ld
else
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_rv.ld
endif
endif

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
SRCS += ${STARTUPFILE}
SRCS += heap.c
ifeq "$(RISCV_CORE)" ""
SRCS += system_$(TARGET_LC).c
else
SRCS += system_riscv_$(TARGET_LC).c
endif
endif

# RISC-V Loader - compile a project for the RISC-V core
# and link it into the same executable as the ARM code
# Configuration Variables:
# - RISCV_LOAD : Set to 1 to enable the RISC-V loader, 0 to disable.  Ex: RISCV_LOAD=1
# - RISCV_APP : Sets the directory of the project
# 				to compile for the RISC-V core.
#				Defaults to "Hello_World".
#				Absolute paths are recommended, but
#				relative paths will also work.
#
# Ex:  "make RISCV_LOAD=1 RISCV_APP=../GPIO"
################################################################################
ifeq ($(RISCV_LOAD),1)

LOADER_SCRIPT := $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/riscv-loader.S

# Directory for RISCV code, defaults to Hello_World
RISCV_APP ?= $(MAXIM_PATH)/Examples/$(TARGET_UC)/Hello_World

# Build the RISC-V app inside of this project so that
# "make clean" will catch it automatically.
# Additionally, set the output directory and filename
# to what the RISC-V loader script expects.
RISCV_BUILD_DIR := $(CURDIR)/build/buildrv

# Binary name for RISCV code.
RISCV_APP_BIN = $(RISCV_BUILD_DIR)/riscv.bin
RISCV_APP_OBJ = $(RISCV_BUILD_DIR)/riscv.o

# Add the RISC-V object to the build.  This is the critical
# line that will get the linker to bring it into the .elf file.
PROJ_OBJS = ${RISCV_APP_OBJ}

.PHONY: rvapp
rvapp: $(RISCV_APP_BIN)

$(RISCV_APP_BIN):
	$(MAKE) -C ${RISCV_APP} BUILD_DIR=$(RISCV_BUILD_DIR) RISCV_CORE=1 RISCV_LOAD=0 PROJECT=riscv
	$(MAKE) -C ${RISCV_APP} BUILD_DIR=$(RISCV_BUILD_DIR) $(RISCV_APP_BIN) RISCV_CORE=1 RISCV_LOAD=0

.PHONY: rvobj
rvobj: $(RISCV_APP_OBJ)

${RISCV_APP_OBJ}: $(LOADER_SCRIPT) ${RISCV_APP_BIN}
	@${CC} ${AFLAGS} -o ${@} -c $(LOADER_SCRIPT)

endif
################################################################################

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

# Include memory definitions
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_memory.mk

# Include the rules and goals for building
ifeq "$(RISCV_CORE)" ""
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/gcc.mk
else
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/gcc_riscv.mk
endif