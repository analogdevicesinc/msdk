# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration



# **********************************************************

# Add the barcode library
IPATH += barcode/include
PROJ_LDFLAGS += -Lbarcode
PROJ_LIBS += barcode

# Add some additional compiler definitions
PROJ_CFLAGS += -DBRCD_RDR_IMG_TO_PC
