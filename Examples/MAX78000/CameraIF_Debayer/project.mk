# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Set the camera driver to the HM0360 color drivers.
# These are the only drivers supported by this example.
CAMERA=HM0360_COLOR

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

ifeq ($(BOARD),FTHR_RevA)
$(error ERR_NOTSUPPORTED: This example is not supported on the MAX78000FTHR!)
endif

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif