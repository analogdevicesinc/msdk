# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# Uncomment the line below to build for the MAX78000FTHR
#BOARD=FTHR_RevA

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

# Add some additional directories to the project based on
# what board we're building for...
ifeq "$(BOARD)" "EvKit_V1"
VPATH += TFT/evkit
endif
ifeq "$(BOARD)" "FTHR_RevA"
VPATH += TFT/fthr
endif

IPATH += TFT/evkit

