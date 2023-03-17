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

# Cordio Library Options
DEBUG           ?= 1
TRACE           ?= 1
BT_VER          ?= 9
INIT_PERIPHERAL ?= 1
INIT_CENTRAL    ?= 1
INIT_ENCRYPTED  ?= 1
INIT_OBSERVER   ?= 0
INIT_BROADCASTER?= 0

WSF_HEAP_SIZE ?= 0x10000
CFG_DEV += WSF_HEAP_SIZE=$(WSF_HEAP_SIZE)

# Select either option, or both for combined Host and Controller on single core
BLE_HOST        ?= 1
BLE_CONTROLLER  ?= 1

ifneq "$(BLE_HOST)" ""
ifneq "$(BLE_HOST)" "0"
ifneq "$(BLE_CONTROLLER)" "1"
RISCV_LOAD = 1
RISCV_APP ?= ../BLE4_ctr
endif
endif
endif

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
PAL_NVM_SIZE	?= 0x2000

CFG_DEV         := BT_VER=$(BT_VER)
CFG_DEV         += SCH_CHECK_LIST_INTEGRITY=1

# 2 = uECC_asm_fast, optimized for speed
CFG_DEV         += uECC_ASM=2

# Configure for both the host and controller by default
BLE_HOST        ?= 1
BLE_CONTROLLER  ?= 1

# Use ExactLE if we're loading both the controller and host
ifneq ($(BLE_HOST),0)
ifneq ($(BLE_CONTROLLER),0)
USE_EXACTLE     := 1
endif
endif

ifeq ($(USE_EXACTLE), 1)
include $(ROOT_DIR)/controller/build/common/gcc/config.mk
endif

# Host includes
ifneq ($(BLE_HOST),0)
include $(ROOT_DIR)/ble-apps/build/common/gcc/config.mk
include $(ROOT_DIR)/ble-apps/build/common/gcc/sources.mk
endif

# Controller only includes
ifneq ($(BLE_CONTROLLER),0)
ifeq ($(BLE_HOST),0)

include $(ROOT_DIR)/controller/build/common/gcc/config.mk
include $(ROOT_DIR)/wsf/build/sources.mk
include $(ROOT_DIR)/platform/build/common/gcc/sources.mk

ifeq ($(BT_VER), 8)
include $(ROOT_DIR)/controller/build/common/gcc/sources_ll_4.mk
else
include $(ROOT_DIR)/controller/build/common/gcc/sources_ll_5.mk
endif

endif
endif

include $(ROOT_DIR)/platform/targets/maxim/build/config_maxim.mk

# APP_BUILD_C_FILES: Rebuild these for each application. This will allow us to limit the code size
# based on the application configuration
ifeq ($(BLE_CONTROLLER),1)
APP_BUILD_C_FILES += ${ROOT_DIR}/controller/sources/ble/init/init_ctr.c
APP_BUILD_C_FILES += ${ROOT_DIR}/controller/sources/ble/init/init.c
endif

# Remove these files from the library build, board level dependencies. Will have to be
# re-built for each application
APP_BUILD_C_FILES += ${ROOT_DIR}/platform/targets/maxim/${CHIP_LC}/sources/pal_uart.c
APP_BUILD_C_FILES += ${ROOT_DIR}/platform/targets/maxim/${CHIP_LC}/sources/pal_sys.c

# This will let us enable/disable trace messaging by application
APP_BUILD_C_FILES += ${ROOT_DIR}/wsf/sources/targets/${RTOS}/wsf_trace.c

# $(filter-out pattern…,text)
C_FILES := $(filter-out ${APP_BUILD_C_FILES},${C_FILES})
