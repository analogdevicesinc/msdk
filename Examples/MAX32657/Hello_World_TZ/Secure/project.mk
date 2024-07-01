# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# TrustZone project with secure and non-secure code.
TRUSTZONE=1

# This is a secure project.
MSECURITY_MODE=SECURE

# Add path to Non-Secure project.
NONSECURE_CODE_DIR=../NonSecure
SECURE_CODE_DIR=.

# Set up custom memory settings if enabled
USE_CUSTOM_MEMORY_SETTINGS = 1

# *_SIZE required if USE_CUSTOM_MEMORY_SETTINGS=1
S_FLASH_SIZE=0x00080000
NS_FLASH_SIZE=0x00080000
S_SRAM_SIZE=0x00020000
NS_SRAM_SIZE=0x00020000

NSC_SIZE=0x00008000 # Not required.

# Optional, but all *_START must be set if used.
# S_FLASH_START=0x11000000
# NS_FLASH_START=0x01080000
# S_SRAM_START=0x30000000
# NS_SRAM_START=0x20032000
