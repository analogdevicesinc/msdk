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
#     Configuration
#--------------------------------------------------------------------------------------------------

ROOT_DIR        ?= $(CORDIO_DIR)
BSP_DIR         ?= $(LIBS_DIR)

ifeq ($(TARGET_UC),MAX32680)
CHIP_UC         ?= MAX32655
CHIP_LC         ?= max32655
else
CHIP_UC         ?= $(TARGET_UC)
CHIP_LC         ?= $(TARGET_LC)
endif

PLATFORM        := maxim
RTOS            ?= baremetal

# Used for storing pairing/bonding information
PAL_NVM_SIZE	?= 0x8000

CFG_DEV         := BT_VER=$(BT_VER)
CFG_DEV         += SCH_CHECK_LIST_INTEGRITY=1

# 2 = uECC_asm_fast, optimized for speed
CFG_DEV         += uECC_ASM=2

# Use the sbrk function for allocating heap space
ifeq ($(RTOS), baremetal)
WSF_HEAP_SBRK   ?= 1
endif

# Use ExactLE if we're loading both the controller and host
ifneq ($(BLE_HOST),)
ifneq ($(BLE_CONTROLLER),)
ifneq ($(BLE_HOST),0)
ifneq ($(BLE_CONTROLLER),0)
USE_EXACTLE     := 1
endif
endif
endif
endif

ifeq ($(USE_EXACTLE), 1)
include $(ROOT_DIR)/controller/build/common/gcc/config.mk
endif

# Host includes
ifneq ($(BLE_HOST),)
include $(CORDIO_DIR)/ble-apps/build/common/gcc/config.mk
include $(CORDIO_DIR)/ble-apps/build/common/gcc/sources.mk
endif

# Controller only includes
ifneq ($(BLE_CONTROLLER),)
ifeq ($(BLE_HOST),)

include $(ROOT_DIR)/controller/build/common/gcc/config.mk
include $(ROOT_DIR)/wsf/build/sources.mk
include $(ROOT_DIR)/platform/build/common/gcc/sources.mk

ifeq ($(BT_VER), 8)
include $(ROOT_DIR)/controller/build/common/gcc/sources_ll_4.mk
else
include $(ROOT_DIR)/controller/build/common/gcc/sources_ll_5.mk
endif

ifeq ($(TRACE),1)
CFG_DEV         += LL_TRACE_ENABLED=1
endif

endif
endif

include $(ROOT_DIR)/platform/targets/maxim/build/config_maxim.mk

# Convert Cordio definitions to Maxim BSP
PROJ_CFLAGS     += $(addprefix -D,$(CFG_DEV))
PROJ_AFLAGS     += -DPAL_NVM_SIZE=$(PAL_NVM_SIZE)
SRCS            += $(C_FILES)
VPATH           += %.c $(sort $(dir $(C_FILES)))
IPATH           += $(INC_DIRS)
