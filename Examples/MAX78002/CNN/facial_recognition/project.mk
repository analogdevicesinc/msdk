# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# This project only supports the Newhaven NHD-2.4 TFT display
TFT = NEWHAVEN

# Enable TFT and touchscreen
PROJ_CFLAGS+=-DTFT_ENABLE
PROJ_CFLAGS+=-DTS_ENABLE
PROJ_CFLAGS +=-DTS_MAX_BUTTONS=32

# Add font
FONTS = LiberationSans16x16
