###################################################################################################
#
# Build make targets
#
# Copyright (c) 2019-2020 Packetcraft, Inc.
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
###################################################################################################

#--------------------------------------------------------------------------------------------------
#     Project
#--------------------------------------------------------------------------------------------------

# Default options
CHIP            ?= max32655
COMPILER        ?= GCC
BOARD           ?= EvKit_V1

TARGET_REV      ?= 0x4131 	# A1 in ASCII

# CPU options
ifeq ($(RISCV_CORE),)
CPU             := cortex-m4
MFLOAT          := softfp
MFPU            := fpv4-sp-d16
else
CPU             := riscv
endif

BSP_DIR         := $(ROOT_DIR)/..
LIBS_DIR        := $(BSP_DIR)
CMSIS_ROOT      := $(LIBS_DIR)/CMSIS

CHIP_UC         := $(shell echo $(CHIP) | tr a-z A-Z)
CHIP_LC         := $(shell echo $(CHIP) | tr A-Z a-z)
TARGET          ?= $(CHIP_UC)
TARGET_NUM      ?= $(shell echo $(TARGET) | tr -dc '0-9')

#--------------------------------------------------------------------------------------------------
#     Configuration
#--------------------------------------------------------------------------------------------------

include $(ROOT_DIR)/platform/targets/maxim/build/config_maxim.mk
CFG_DEV         += TARGET=$(TARGET_NUM)
CFG_DEV         += TARGET_REV=$(TARGET_REV)

#--------------------------------------------------------------------------------------------------
#     Sources
#--------------------------------------------------------------------------------------------------

# Linker file
ifeq ($(LD_FILE),)
ifeq ($(RISCV_CORE),)
LD_FILE         := $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/$(COMPILER)/$(CHIP_LC).ld
else
LD_FILE         := $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/$(COMPILER)/$(CHIP_LC)_riscv.ld
endif
endif

#--------------------------------------------------------------------------------------------------
#     Compilation flags
#--------------------------------------------------------------------------------------------------

ifeq ($(RISCV_CORE),)
# ARM flags
# Compiler flags
C_FLAGS         += -mcpu=$(CPU) -mthumb -mlittle-endian
C_FLAGS         += -mfloat-abi=$(MFLOAT) -mfpu=$(MFPU) 

A_FLAGS         += -mcpu=$(CPU) -mthumb -mlittle-endian
A_FLAGS         += -mfloat-abi=$(MFLOAT) -mfpu=$(MFPU) -MD                                                                

# Linker flags
LD_FLAGS        += -mthumb -mcpu=$(CPU)
LD_FLAGS        += -mfloat-abi=$(MFLOAT) -mfpu=$(MFPU) --entry=Reset_Handler

else
#RISCV flags
C_FLAGS 	+= -march=rv32imc -mabi=ilp32 -c
LD_FLAGS 	+= -march=rv32imafdc
endif
