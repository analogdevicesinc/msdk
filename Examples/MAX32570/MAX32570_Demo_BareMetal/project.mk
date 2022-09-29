# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Override default optimization level
MXC_OPTIMIZE_CFLAGS=-O1

# Enable NFC and EMV libraries
LIB_NFC = 1
LIB_EMV = 1

# Enable SBT
SBT = 1

# Add some compiler flags specific to the NFC and EMV libs
PROJ_CFLAGS += -D__$(TARGET_UC)

# Set SDMA size
PROJ_AFLAGS += -D__MSR_SDMA_SIZE=0xA000

# Add project's include and source paths
VPATH += src/nfc
VPATH += src/nfc/contactless_l1_app
VPATH += src/nfc/emv_l1_stack
VPATH += src/msr
VPATH += resources
IPATH += include/nfc
IPATH += include/nfc/contactless_l1_app
IPATH += include/nfc/emv_l1_stack
IPATH += resources
