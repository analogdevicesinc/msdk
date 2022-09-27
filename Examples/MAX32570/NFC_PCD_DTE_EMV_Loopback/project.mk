# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Override default optimization level
MXC_OPTIMIZE_CFLAGS=-O1

ifeq ("$(wildcard $(LIBS_DIR)/NFC)","")
$(error This demo example requires NFC package. Please install the NFC package to Libraries/NFC.)
endif

ifeq ("$(wildcard $(LIBS_DIR)/EMV)","")
$(error This demo example requires EMV package. Please install the EMV package to Libraries/EMV.)
endif

# Enable NFC library
LIB_NFC = 1

# Add project's include and source paths
VPATH += src/nfc
VPATH += src/nfc/contactless_l1_app
VPATH += src/nfc/emv_l1_stack
VPATH += resources
IPATH += include/nfc
IPATH += include/nfc/contactless_l1_app
IPATH += include/nfc/emv_l1_stack
IPATH += resources
