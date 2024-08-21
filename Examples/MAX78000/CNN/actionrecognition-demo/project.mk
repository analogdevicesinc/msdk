# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

BOARD = FTHR_RevA

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

FONTS = LiberationSans12x12 LiberationSans16x16

ifneq "$(BOARD)" "FTHR_RevA"
$(error ERR_NOTSUPPORTED: This project is only supported for the MAX78000FTHR board.)
endif

