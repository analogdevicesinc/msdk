###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
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

# This file helps set up the memory settings for the project.
#	Due to the nature of TrustZone, this was added to help
#	project-owners conveniently set up their project's memory
#	settings in their project.mk file to meet their application
#	requirements without having to go into multiple files:
#		- {device}.h
#		- linker files
#		- partition_{device}.h
#
#	Of course, at the cost of more overhead.

# Physical Memory Settings
PHY_FLASH_START ?= 0x01000000
PHY_FLASH_SIZE ?= 0x00100000 # 1MiB

PHY_SRAM_START ?= 0x20000000
PHY_SRAM_SIZE ?= 0x00040000 # 256KiB

###############################################################################
# Required arguments that user must set in project.mk if they choose to
# use their own memory settings.
#

USE_CUSTOM_MEMORY_SETTINGS ?= 0

# These four are required if USE_CUSTOM_MEMORY_SETTING is set to 1.
# 	The sum of the Flash region sizes must equal
#	the total physical Flash size (1MiB).
S_FLASH_SIZE ?= 0x00080000 # Secure Region default: 512KiB
NS_FLASH_SIZE ?= 0x00080000 # Non-Secure Region default: 512KiB

# 	The sum of the SRAM region sizes must equal
#	the total physical SRAM size (256KiB).
S_SRAM_SIZE ?= 0x00020000 # Secure Region default: 128KiB
NS_SRAM_SIZE ?= 0x00020000 # Non-Secure Region default: 128KiB

# Sets which type of memory the main code executes from.
# Only possible options with this is: FLASH or SRAM.
EXECUTE_CODE_MEM ?= FLASH

###############################################################################
# Optional add-on arguments that the user can set in project.mk
#	for finer memory control if USE_CUSTOM_MEMORY_SETTINGS is set to 1.
#
# Extra overhead, but this is setup so the project-owners can just set up
#	the sizes, if they don't need this much control with memory
#
# The project-owner does not have to set these arguments if they choose
#	not to; however, if one of these are set, then they ALL must be set.
#
# The integer arguments are set to 0 by default, so the setup_memory_tz.py
#	script knows if the user has updates these settings or not.
#

S_FLASH_START ?= 0
NS_FLASH_START ?= 0

S_SRAM_START ?= 0
NS_SRAM_START ?= 0

# Size of Non-Secure Callable Region.
#	This region will be placed at the end of the Secure Region.
#	Which Secure Region memory type (Flash or SRAM) the NSC
#	will be in is determined by EXECUTE_CODE_MEM
# The NSC region is still considered part of the Secure Region.
#   NSC_SIZE determines how much space of the Secure Region that
#	the NSC region will use.
NSC_SIZE ?= 0

###############################################################################
# Generate linker scripts and partition_max32657.h if project-owner decides
#	to use custom memory settings.
#
#
ifeq ($(TRUSTZONE),1)

# This is added to prevent the script from running twice because the "Secure"
#	project is re-built in order to generate the CMSE import library object file.
#	The Secure project needs to fully build to generate the CMSE import
#	library object file that Non-Secure code requires.
ifeq ($(GEN_CMSE_IMPLIB_OBJ),0)

ifeq ($(USE_CUSTOM_MEMORY_SETTINGS),1)

ifneq "$(PYTHON_CMD)" "none"

# For formatting python script output. Do not change contents of this define!!
define NEWLINE


endef

# This section was not placed in a recipe because the partition_max32657.h file needs to be
#	generated/updated before the entired project builds. This is the order which
#	TrustZone-enabled projects build:
#		1. Generate/Update Linker Scripts and partition_max32657.h
#		2. Compile the Secure project as the "Host" project, but do not call linker.
#			That is done at the end.
#		3. Fully build the Secure project (compile AND link) individually to generate
#			the CMSE import library object file required for Non-Secure Code.
#			The CMSE import library object file is generated when the linker is called.
#		4. Compile the Non-Secure project as the "Target" project, and call the linker
#			with the CMSE import library object file to create the Non-Secure code image.
#		5. Call the linker to combine the Secure and Non-Secure images together.
#
# Downside is that this script will be called when you run "make clean" when the project
#	is set up to use custom memory settings.
TZ_SCRIPT = $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/setup_memory_tz.py

TZ_SCRIPT_ARGS = $(MAXIM_PATH) $(abspath $(SECURE_CODE_DIR)) $(abspath $(NONSECURE_CODE_DIR)) $(S_FLASH_SIZE) $(NS_FLASH_SIZE) $(S_SRAM_SIZE) $(NS_SRAM_SIZE) $(NSC_SIZE) $(S_FLASH_START) $(NS_FLASH_START) $(S_SRAM_START) $(NS_SRAM_START) $(EXECUTE_CODE_MEM)

TZ_SCRIPT_OUTPUT := $(shell $(PYTHON_CMD) $(TZ_SCRIPT) $(TZ_SCRIPT_ARGS))

$(info $(subst > ,$(NEWLINE),$(TZ_SCRIPT_OUTPUT)))

TZ_SCRIPT_ERROR = $(findstring ERROR,$(TZ_SCRIPT_OUTPUT))
ifneq "$(TZ_SCRIPT_ERROR)" ""
$(error Error preparing linker scripts and partition_max32657.h file. Check memory settings in project.mk)
endif # TZ_SCRIPT_ERROR

else # PYTHON_CMD
$(error $(PYTHON_CMD) No Python installation detected on your system!  Could not auto-generate custom linker scripts.  Please manually update the linker scripts)
endif # PYTHON_CMD

endif # USE_CUSTOM_MEMORY_SETTINGS
endif # GEN_CMSE_IMPLIB_OBJ
endif # TRUSTZONE

###############################################################################
# Setup Device Memory Settings (usually in max32657.h).
#

# For secure/non-secure partitioned projects.
ifeq ($(TRUSTZONE),1)

ifeq "$(MSECURITY_MODE)" "SECURE"

ifeq ($(USE_CUSTOM_MEMORY_SETTINGS),1)
# After script is done, write the final memory settings to be used as definitions in your project.
#	This is done afterwards so the script knows when to use default arguments (*_START variables).
# If default settings are used (no *_START variable is set in project.mk), then, like the TZ_SCRIPT
#	the secure regions will start at the lower half of memory.
# Note, all *_START variables must be set by user (not default 0 value) or an error is thrown.
REGION_START_TOTAL = $(shell echo $$(( $(S_FLASH_START) + $(NS_FLASH_START) + $(S_SRAM_START) + $(NS_SRAM_START) )))
ifeq ($(REGION_START_TOTAL),0)

S_FLASH_START := $(shell echo $$(( $(PHY_FLASH_START) | (1<<28) )))

NS_FLASH_START := $(shell echo $$(( $(PHY_FLASH_START) + $(S_FLASH_SIZE) )))

S_SRAM_START := $(shell echo $$(( $(PHY_SRAM_START) | (1<<28) )))

NS_SRAM_START := $(shell echo $$(( $(PHY_SRAM_START) + $(S_SRAM_SIZE) )))

endif # REGION_START_TOTAL

# If user configured all settings, then they should already be prepared to store in PROJ_CFLAGS.

else # !USE_CUSTOM_MEMORY_SETTINGS (not configured by user.)

# Do not use custom memory settings, use the default.
S_FLASH_START := $(shell echo $$(( $(PHY_FLASH_START) | (1<<28) )))
S_FLASH_SIZE := $(shell echo $$(( $(PHY_FLASH_SIZE) / 2 )))

NS_FLASH_START := $(shell echo $$(( $(PHY_FLASH_START) + $(S_FLASH_SIZE) )))
NS_FLASH_SIZE := $(shell echo $$(( $(PHY_FLASH_SIZE) / 2 )))

S_SRAM_START := $(shell echo $$(( $(PHY_SRAM_START) | (1<<28) )))
S_SRAM_START := $(shell printf "0x%x" $(S_SRAM_START))
S_SRAM_SIZE := $(shell echo $$(( $(PHY_SRAM_SIZE) / 2 )))

NS_SRAM_START := $(shell echo $$(( $(PHY_SRAM_START) + $(S_SRAM_SIZE) )))
NS_SRAM_SIZE := $(shell echo $$(( $(PHY_SRAM_SIZE) / 2 )))

endif # USE_CUSTOM_MEMORY_SETTINGS

# Prepare definitions to store in PROJ_CFLAGS.
S_FLASH_START := $(shell printf "0x%x" $(S_FLASH_START))
S_FLASH_SIZE := $(shell printf "0x%x" $(S_FLASH_SIZE))

NS_FLASH_START := $(shell printf "0x%x" $(NS_FLASH_START))
NS_FLASH_SIZE := $(shell printf "0x%x" $(NS_FLASH_SIZE))

S_SRAM_START := $(shell printf "0x%x" $(S_SRAM_START))
S_SRAM_SIZE := $(shell printf "0x%x" $(S_SRAM_SIZE))

NS_SRAM_START := $(shell printf "0x%x" $(NS_SRAM_START))
NS_SRAM_SIZE := $(shell printf "0x%x" $(NS_SRAM_SIZE))

# Set up definitions that will be used in max32657.h
__MXC_FLASH_MEM_BASE=$(S_FLASH_START)
__MXC_FLASH_MEM_SIZE=$(S_FLASH_SIZE)

__MXC_SRAM_MEM_BASE=$(S_SRAM_START)
__MXC_SRAM_MEM_SIZE=$(S_SRAM_SIZE)

# Secure code can have knowledge of Non-Secure regions.
__MXC_FLASH_NS_MEM_BASE=$(NS_FLASH_START)
__MXC_FLASH_NS_MEM_SIZE=$(NS_FLASH_SIZE)

__MXC_SRAM_NS_MEM_BASE=$(NS_SRAM_START)
__MXC_SRAM_NS_MEM_SIZE=$(NS_SRAM_SIZE)

# Set non-secure definitons for Secure code.
PROJ_CFLAGS += -D__MXC_FLASH_NS_MEM_BASE=$(__MXC_FLASH_NS_MEM_BASE)
PROJ_CFLAGS += -D__MXC_FLASH_NS_MEM_SIZE=$(__MXC_FLASH_NS_MEM_SIZE)

PROJ_CFLAGS += -D__MXC_SRAM_NS_MEM_BASE=$(__MXC_SRAM_NS_MEM_BASE)
PROJ_CFLAGS += -D__MXC_SRAM_NS_MEM_SIZE=$(__MXC_SRAM_NS_MEM_SIZE)

else # MSECURITY_MODE=NONSECURE

# Formatting to hex for readability sake.
NS_FLASH_START := $(shell printf "0x%x" $(NS_FLASH_START))
NS_FLASH_SIZE := $(shell printf "0x%x" $(NS_FLASH_SIZE))

NS_SRAM_START := $(shell printf "0x%x" $(NS_SRAM_START))
NS_SRAM_SIZE := $(shell printf "0x%x" $(NS_SRAM_SIZE))

# Check max32657.mk
# When non-secure project builds, the "host" secure project passes in these
#	variables (NS_*_START and NS_*_SIZE).
__MXC_FLASH_MEM_BASE=$(NS_FLASH_START)
__MXC_FLASH_MEM_SIZE=$(NS_FLASH_SIZE)

__MXC_SRAM_MEM_BASE=$(NS_SRAM_START)
__MXC_SRAM_MEM_SIZE=$(NS_SRAM_SIZE)

endif # MSECURITY_MODE

else # TRUSTZONE disabled

# bit 28 of address indicates the security access.
#	0: Non-Secure
#	1: Secure
# Default, startup state is Secure
__MXC_FLASH_MEM_BASE := $(shell echo $$(( $(PHY_FLASH_START) | (1<<28) )))
__MXC_FLASH_MEM_BASE := $(shell printf "0x%x" $(__MXC_FLASH_MEM_BASE))
__MXC_FLASH_MEM_SIZE := $(shell printf "0x%x" $(PHY_FLASH_SIZE))

__MXC_SRAM_MEM_BASE := $(shell echo $$(( $(PHY_SRAM_START) | (1<<28) )))
__MXC_SRAM_MEM_BASE := $(shell printf "0x%x" $(__MXC_SRAM_MEM_BASE))
__MXC_SRAM_MEM_SIZE := $(shell printf "0x%x" $(PHY_SRAM_SIZE))

endif

# Set the definitions
PROJ_CFLAGS += -D__MXC_FLASH_MEM_BASE=$(__MXC_FLASH_MEM_BASE)
PROJ_CFLAGS += -D__MXC_FLASH_MEM_SIZE=$(__MXC_FLASH_MEM_SIZE)

PROJ_CFLAGS += -D__MXC_SRAM_MEM_BASE=$(__MXC_SRAM_MEM_BASE)
PROJ_CFLAGS += -D__MXC_SRAM_MEM_SIZE=$(__MXC_SRAM_MEM_SIZE)

# Should only be run from the host "Secure" project
.PHONY: debug_tz_mem_from_host_proj
debug_tz_mem_from_host_proj:
	@echo
	@echo 'debug_tz_mem_from_host_proj' only works from "Host" or "Secure" project.
	@echo
	@echo S_FLASH_START = ${S_FLASH_START}
	@echo S_FLASH_SIZE = ${S_FLASH_SIZE}
	@echo
	@echo NS_FLASH_START = ${NS_FLASH_START}
	@echo NS_FLASH_SIZE = ${NS_FLASH_SIZE}
	@echo
	@echo S_SRAM_START = ${S_SRAM_START}
	@echo S_SRAM_SIZE = ${S_SRAM_SIZE}
	@echo
	@echo NS_SRAM_START = ${NS_SRAM_START}
	@echo NS_SRAM_SIZE = ${NS_SRAM_SIZE}
	@echo
	@echo NSC_SIZE = ${NSC_SIZE}
	@echo
	@echo EXECUTE_CODE_MEM = ${EXECUTE_CODE_MEM}
	@echo
	@echo USE_CUSTOM_MEMORY_SETTINGS = ${USE_CUSTOM_MEMORY_SETTINGS}
	@echo GEN_CMSE_IMPLIB_OBJ = ${GEN_CMSE_IMPLIB_OBJ}
	@echo MSECURITY_MODE = ${MSECURITY_MODE}
	@echo TRUSTZONE = ${TRUSTZONE}
	@echo
	@echo SECURE_CODE_DIR = $(abspath ${SECURE_CODE_DIR})
	@echo NONSECURE_CODE_DIR = $(abspath ${NONSECURE_CODE_DIR})
	@echo LINKERFILE = $(abspath $(LINKERFILE))
