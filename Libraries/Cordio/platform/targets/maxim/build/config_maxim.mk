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

BAUD            ?= 115200
HWFC            ?= 0
STANDBY_ENABLED ?= 0

# Include chip specific config
include $(ROOT_DIR)/platform/targets/maxim/$(CHIP_LC)/build/config.mk

# Peripherals
CFG_DEV         += BB_CLK_RATE_HZ=1000000
CFG_DEV         += LCTR_CONN_NO_TIFS_REASSEMBLY=1
CFG_DEV         += $(CHCI_TR)
CFG_DEV         += UART_BAUD=$(BAUD)
CFG_DEV         += UART_HWFC=$(HWFC)
CFG_DEV         += BB_ENABLE_INLINE_ENC_TX=1
CFG_DEV         += BB_ENABLE_INLINE_DEC_RX=1
CFG_DEV         += PAL_MAX_RTC_COUNTER_VAL=0xFFFFFFFF

# Schedule the DTM RX due time based on previously received packets
CFG_DEV         += DTM_RX_SCHEDULING=1

# Enable standby mode when idle
# LL_WW_RX_DEVIATION_USEC is a constant defined in the Bluetooth specification
# Set to 0 to improve RX performance, set to 16 to comply with the specification
# and reduce chances of synchronization loss. Accuracy of the 32 kHz crystal is critical
# to performance when using standby mode.
CFG_DEV         += LL_WW_RX_DEVIATION_USEC=0

# Third party software config
ifneq ($(RISCV_CORE),)
CFG_DEV         += uECC_PLATFORM=uECC_arch_other
endif

# Board
CFG_DEV         += BOARD=$(BOARD)
