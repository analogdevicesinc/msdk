###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
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

OLD_GCC_RISCV_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# File has moved to a central location.  The section below preserves legacy projects.
ifeq "$(MAXIM_PATH)" ""
include $(OLD_GCC_RISCV_DIR )/../../../GCC/gcc_riscv.mk
else
include $(MAXIM_PATH)/Libraries/CMSIS/Device/Maxim/GCC/gcc_riscv.mk
endif
