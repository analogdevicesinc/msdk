###################################################################################################
#
# CHIP build options
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

ifneq ($(RISCV_LOAD),)
ifneq ($(RISCV_LOAD),0)
ifneq ($(RISCV_CORE),)
ifneq ($(RISCV_CORE),0)
$(error Cannot have RISCV_LOAD and RISCV_CORE defined)
endif
endif
endif
endif

ifneq ($(RISCV_LOAD),)
ifneq ($(RISCV_LOAD),0)
CFG_DEV         += PAL_SYS_RISCV_LOAD=1
HCI_TR_MAIL     := 1
endif
endif

ifneq ($(RISCV_CORE),)
ifneq ($(RISCV_CORE),0)
HCI_TR_MAIL     := 1
endif
endif

# Use the shared memory HCI transport 
ifneq ($(HCI_TR_MAIL),)
CFG_DEV 				+= HCI_TR_MAIL=1
endif
