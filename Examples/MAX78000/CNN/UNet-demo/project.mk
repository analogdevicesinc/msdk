# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

# Add some additional directories to the build based on the
# board we're compiling for...
ifeq "$(BOARD)" "EvKit_V1"
IPATH += TFT/evkit/
VPATH += TFT/evkit/
endif

ifeq "$(BOARD)" "FTHR_RevA"
IPATH += TFT/fthr
VPATH += TFT/fthr
endif

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif

