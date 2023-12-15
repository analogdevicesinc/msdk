# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# This example is only compatible with the FTHR board,
# so we override the BOARD value to hard-set it.
override BOARD=EvKit_V1
$(warning Warning: This project is forced to compile for the EvKit_V1 board only!)

$(info Note: This project is designed and tested for the NewHaven NHD-2.4 screen only!)
#LIB_LVGL = 1
#TFT = ADAFRUIT
TFT=NEWHAVEN
# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
PROJ_CFLAGS+=-DTFT_ENABLE
PROJ_CFLAGS+=-DTS_ENABLE
PROJ_CFLAGS +=-DTS_MAX_BUTTONS=32
IPATH += TFT/fthr
VPATH += TFT/fthr
endif



