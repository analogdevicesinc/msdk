###################################################################################################
#
# Source and include definition
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
#   Includes
#--------------------------------------------------------------------------------------------------

INC_DIRS  += \
	$(BSP_DIR)/Boards/$(CHIP_UC)/Include \
	$(BSP_DIR)/Boards/$(CHIP_UC)/$(BOARD)/Include \
	$(BSP_DIR)/PeriphDrivers/Include/$(CHIP_UC) \
	$(BSP_DIR)/CMSIS/Include \
	$(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Include

#--------------------------------------------------------------------------------------------------
#	BSP
#--------------------------------------------------------------------------------------------------

ifeq ($(PERIPH_DRIVER_LIB),)

ifeq ($(RISCV_CORE),)
C_FILES 	+= $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/system_$(CHIP_LC).c
else
C_FILES 	+= $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/system_riscv_$(CHIP_LC).c
endif

ifeq ($(RISCV_CORE),)
A_FILES 	+= $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/$(COMPILER)/startup_$(CHIP_LC).S
else
A_FILES 	+= $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/$(COMPILER)/startup_riscv_$(CHIP_LC).S
endif

-include $(BSP_DIR)/CMSIS/Device/Maxim/$(CHIP_UC)/Source/$(COMPILER)/$(CHIP_LC)_memory.mk
A_FLAGS += $(PROJ_AFLAGS)

C_FILES   += \
	$(BSP_DIR)/Boards/$(CHIP_UC)/$(BOARD)/Source/board.c \
	$(BSP_DIR)/Boards/$(CHIP_UC)/Source/led.c \
	$(BSP_DIR)/Boards/$(CHIP_UC)/Source/pb.c

include $(BSP_DIR)/PeriphDrivers/$(CHIP_LC)_files.mk

C_FILES 	+= \
	$(PERIPH_DRIVER_C_FILES)

A_FILES 	+= \
	$(PERIPH_DRIVER_A_FILES)
endif

INC_DIRS 	+= \
	$(PERIPH_DRIVER_INCLUDE_DIR)

ifneq ($(USE_EXACTLE),0)
ifeq ($(RISCV_CORE),)
LIBS      += $(BSP_DIR)/BlePhy/$(CHIP_UC)/libphy.a
else
LIBS      += $(BSP_DIR)/BlePhy/$(CHIP_UC)/libphy_riscv.a
endif
endif
