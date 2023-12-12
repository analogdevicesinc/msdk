###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 # (now owned by Analog Devices, Inc.)
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
 ##############################################################################
 #
 # Copyright 2023 Analog Devices, Inc.
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

ifeq "$(CMSIS_ROOT)" ""
# If CMSIS_ROOT is not specified, this Makefile will calculate CMSIS_ROOT relative to itself.
GCC_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CMSIS_ROOT := $(abspath $(GCC_DIR)../../../../..)
endif

TARGET_UC:=MAX78000
TARGET_LC:=max78000

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

################################################################################
# Detect target OS
# windows : native windows
# windows_msys : MSYS2 on windows
# windows_cygwin : Cygwin on windows (legacy config from old sdk)
# linux : Any linux distro
# macos : MacOS
ifeq "$(OS)" "Windows_NT"
_OS = windows

UNAME_RESULT := $(shell uname -s 2>&1)
# MSYS2 may be present on Windows.  In this case,
# linux utilities should be used.  However, the OS environment
# variable will still be set to Windows_NT since we configure
# MSYS2 to inherit from Windows by default.
# Here we'll attempt to call uname (only present on MSYS2)
# while routing stderr -> stdout to avoid throwing an error 
# if uname can't be found.
ifneq ($(findstring CYGWIN, $(UNAME_RESULT)), )
CYGWIN=True
_OS = windows_cygwin
endif

ifneq ($(findstring MSYS, $(UNAME_RESULT)), )
MSYS=True
_OS = windows_msys
endif
ifneq ($(findstring MINGW, $(UNAME_RESULT)), )
MSYS=True
_OS = windows_msys
endif

else # OS

UNAME_RESULT := $(shell uname -s)
ifeq "$(UNAME_RESULT)" "Linux"
_OS = linux
endif
ifeq "$(UNAME_RESULT)" "Darwin"
_OS = macos
endif

endif

################################################################################

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
RISCV_APP ?= $(CMSIS_ROOT)/../../Examples/$(TARGET_UC)/Hello_World

# Build the RISC-V app inside of this project so that
# "make clean" will catch it automatically.
# Additionally, set the output directory and filename
# to what the RISC-V loader script expects.
RISCV_BUILD_DIR := $(BUILD_DIR)/buildrv

# Binary name for RISCV code.
RISCV_APP_BIN = $(RISCV_BUILD_DIR)/riscv.bin

# Add the RISC-V object to the build.  This is the critical
# line that will get the linker to bring it into the .elf file.
RISCV_APP_OBJ = $(RISCV_BUILD_DIR)/riscv.o
PROJ_OBJS = ${RISCV_APP_OBJ}

# To append the RISC-V binary to the Arm binary correctly, we need to
# offset the flash start address when we link the RISC-V binary.

# The first challenge is automatically detecting the size of the arm binary.
# We do that by defining a symbol in the primary linkerfile (max78000.ld) called "_riscv_boot".
# Once linked, we can parse the .map file for the memory location of this symbol.
# This method is also nice because we can reference "_riscv_boot" in our C code 
# to automagically set the RISC-V boot address correctly.
ARM_MAP_FILE = $(BUILD_DIR)/$(PROJECT).map
.PHONY: armmap
armmap: $(ARM_MAP_FILE)

$(ARM_MAP_FILE):
# The rule to build the arm-only map file re-builds the project with RISCV_LOAD set to 0.
	$(MAKE) -C $(CURDIR) RISCV_LOAD=0 PROJECT=$(PROJECT)

# Linker scripts unfortunately do not accept environment variables
# or compiler definitions as input.  The only way to parameterize a
# linkerfile is to include a separate generated linkerfile, which
# complicates things significantly.  The RISC-V linkerfile (max78000_riscv.ld)
# has a hard-coded INCLUDE "buildrv/common_riscv.ld" to look for this file.
RISCV_COMMON_LD = $(BUILD_DIR)/buildrv/common_riscv.ld
PROJ_LDFLAGS += -L$(abspath $(BUILD_DIR))
# ^ Add to search path to locate buildrv/common_riscv.ld

.PHONY: rvcommonld
rvcommonld: $(RISCV_COMMON_LD)

# This is where the implementation gets complicated...  but what we want to do is simple.
# 1) Parse the memory address of the "_riscv_boot" symbol.
# 2) Write definitions for __FlashStart and __FlashLength in the auto-generated linkerfile.
.NOTPARALLEL: $(RISCV_COMMON_LD)
# ^ Note ".NOTPARALLEL" is used.  Despite explicitly listing the arm map file as a dependency,
# parallel builds would sometimes break this target.  
# (https://www.gnu.org/software/make/manual/make.html#Parallel-Disable)
$(RISCV_COMMON_LD): $(ARM_MAP_FILE)
	$(info - Detecting Arm code size and generating $(@))
	$(info - Input file: $(ARM_MAP_FILE))
	$(info - Output file: $(@))
ifeq "$(_OS)" "windows"
# Forgive me...
	@powershell -Command "New-Item -Force -Path \"$(dir $(@))\" -ItemType \"directory\" | Out-Null"
# 	^ Create the directory for the file
	@powershell -Command "Out-File -FilePath $(@) -Encoding \"ascii\" -InputObject (-join(\"__FlashStart = \", ((Get-Content $(<) | Select-String -Pattern _riscv_boot).Line.Trim() -Split \" \")[0], \";\"))"
# 	This one-liner was generally constructed in the following order
#	- Read the file with Get-Content
#	- Perform the pattern matching with Select-String.  This gives us a Match object.  The .Line attribute gives us the whole line, including whitespace
#	- Use the string object's built-in Trim() method to remove leading/extra whitespace.
#	- Now we can split on the space with the -Split command, and the address is the first entry in the array.
#	- We have the address as a string now.  We need to parse it into another string like "__FlashStart = 0x10040000;"  The -join function seems the best way to keep this a one-liner...  It joins all given strings together.
#	- Write to the file with Out-File.  Note the explicit 'ascii' encoding.  UTF-8 has issues injecting a BOM (Byte Order Mark) and the linker will throw a warning about \357\273\277 chars.  utf8NoBOM is listed as an encoding option in the documentation, but PowerShell throws an error... ascii it is!
	@powershell -Command "Out-File -FilePath $(@) -Encoding \"ascii\" -Append -InputObject \"__FlashLength = 0x10080000 - __FlashStart;\""
else
# Linux/MacOS
	@mkdir -p $(dir $(@)) && touch $(@)
	@echo "__FlashStart = $(shell grep -o "0x[[:alnum:]]*[[:blank:]]*_riscv_boot = ." $(<) | cut -d " " -f 1);" > $(@)
#   ^ grep gives us the line with a regex, and the "-o" gives us an exact match that strips some extra whitespace.
# 	Then, we cut the line on the space delimiter and save the first entry.
	@echo "__FlashLength = 0x10080000 - __FlashStart;" >> $(@)
endif

# Now we can build and link the RISC-V app, and then create an object file that the linker can
# use to combine it with the Arm code.  Given that the two binaries share different instruction
# sets, that's also not straightforward.  We leverage the ".incbin" assembly instruction, which
# allows us to create a "wrapper" object file around the raw RISC-V binary.  
.PHONY: rvapp
rvapp: $(RISCV_APP_BIN)

$(RISCV_APP_BIN): $(RISCV_COMMON_LD)
# Build the RISC-V project
	@$(MAKE) -C ${RISCV_APP} BUILD_DIR=$(RISCV_BUILD_DIR) RISCV_CORE=1 RISCV_LOAD=0 PROJECT=riscv HOST_PROJECT=$(PROJECT) PROJ_LDFLAGS="$(PROJ_LDFLAGS)"
# Create the binary (should incrementally build off of the first pass for the .elf)
	@$(MAKE) -C ${RISCV_APP} $(RISCV_APP_BIN) BUILD_DIR=$(RISCV_BUILD_DIR) RISCV_CORE=1 RISCV_LOAD=0 PROJ_LDFLAGS="$(PROJ_LDFLAGS)"

# Create the object file
.PHONY: rvobj
rvobj: $(RISCV_APP_OBJ)

${RISCV_APP_OBJ}: $(LOADER_SCRIPT) $(RISCV_APP_BIN)
	@${CC} ${AFLAGS} -o ${@} -c $(LOADER_SCRIPT)

else # RISCV_LOAD
# This condition is hit when building a RISCV project as "standalone" (ie. make RISCV_CORE=1) without using RISCV_LOAD
# In this case, we still need to provide the linker
# with a generated input file.  We give the RISCV
# complete ownership of flash here.  Note that 
# the RISC-V core still needs to be "spun up" by
# the Arm core, so this is really only useful for
# being able to compile and link quickly for RISC-V.
RISCV_COMMON_LD = $(BUILD_DIR)/common_riscv.ld
PROJ_LDFLAGS += -L$(abspath $(BUILD_DIR))
# ^ Add to search path to locate buildrv/common_riscv.ld

$(RISCV_COMMON_LD):
	$(info - Generating $(@))
ifeq "$(_OS)" "windows"
	@powershell -Command "New-Item -Force -Path \"$(dir $(@))\" -ItemType \"directory\" | Out-Null"
	@powershell -Command "Out-File -FilePath $(@) -Encoding \"ascii\" -InputObject \"__FlashStart = 0x10000000;\""
	@powershell -Command "Out-File -FilePath $(@) -Encoding \"ascii\" -Append -InputObject \"__FlashLength = 0x10080000 - __FlashStart;\""
else
	@mkdir -p $(dir $(@)) && touch $(@)
	@echo "__FlashStart = 0x10000000;" > $(@)
	@echo "__FlashLength = 0x10080000 - __FlashStart;" >> $(@)
endif

endif
################################################################################

ifeq "$(LINKERFILE)" ""
ifeq "$(RISCV_CORE)" ""
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC).ld
else
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_riscv.ld
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

# Select the xpack toolchain to use
RISCV_PREFIX ?= riscv-none-elf

# Include the rules and goals for building
ifeq "$(RISCV_CORE)" ""
include $(CMSIS_ROOT)/Device/Maxim/GCC/gcc.mk
else
include $(CMSIS_ROOT)/Device/Maxim/GCC/gcc_riscv.mk
endif

# Include rules for flashing
include $(CMSIS_ROOT)/../../Tools/Flash/flash.mk
