###############################################################################
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
ifeq ($(RISCV_LOAD),1)

LOADER_SCRIPT := $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/riscv-loader.S

# Directory for RISCV code, defaults to Hello_World
RISCV_APP ?= $(CMSIS_ROOT)/../../Examples/$(TARGET_UC)/Hello_World

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
	$(MAKE) -C ${RISCV_APP} BUILD_DIR=$(RISCV_BUILD_DIR) RISCV_CORE=1 RISCV_LOAD=0 PROJECT=riscv HOST_PROJECT=$(PROJECT)
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
