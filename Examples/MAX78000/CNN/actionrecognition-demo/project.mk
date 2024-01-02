# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

# Add some additional directories to the build based on what board
# we're compiling for...
ifeq "$(BOARD)" "EvKit_V1"
VPATH += TFT/evkit
endif
ifeq "$(BOARD)" "FTHR_RevA"
VPATH += TFT/fthr/fonts
endif

$(info $$var is [${BOARD}])
 IPATH += TFT/evkit

FONTS = LiberationSans12x12 LiberationSans24x24 LiberationSans28x28 LiberationSans16x16

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif