###############################################################################
 #
 # Copyright (C) 2024-2025 Analog Devices, Inc.
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

# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
MXC_OPTIMIZE_CFLAGS = -O0

# TrustZone project with secure and non-secure code.
TRUSTZONE=1

# This is a secure project.
MSECURITY_MODE=SECURE

# Add path to Non-Secure project.
NONSECURE_CODE_DIR=../NonSecure
SECURE_CODE_DIR=.

# Set up custom memory settings if enabled
#	1 - enabled, 0 - disabled.
USE_CUSTOM_MEMORY_SETTINGS = 0

################################################################################
#
#	If 'USE_CUSTOM_MEMORY_SETTINGS = 1':
#

# Select which type of memory the main code will execute from.
#	Possible options: FLASH or SRAM
EXECUTE_CODE_MEM=FLASH

# *_SIZE required if USE_CUSTOM_MEMORY_SETTINGS=1
# Total of *_FLASH_SIZE and *_SRAM_SIZE regions must equal their respective physical size constraints.
#	FLASH: 1MB = 0x00100000
#	SRAM: 256KB = 0x00040000
S_FLASH_SIZE=0x00080000
NS_FLASH_SIZE=0x00080000
S_SRAM_SIZE=0x00020000
NS_SRAM_SIZE=0x00020000

# Depending on what type of memory the main code will execute from, a portion
#	of the secure region would be used for the Non-Secure Callable Region.
# This sets the size of that region.
NSC_SIZE=0x00008000 # Not required.

# Optional for finer grain control, but all *_START variables must be set if provided.
# Note: bit 28 of the starting address indicates the security state of the region.
#	If not provided, by default, the secure regions start in first half of memory, and the non-
#	secure regions start in the second half.
# S_FLASH_START=0x11000000
# NS_FLASH_START=0x01080000
# S_SRAM_START=0x30000000
# NS_SRAM_START=0x20020000
