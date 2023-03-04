# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Set the camera drivers.  Select a line to match the
# connected camera.  These are some common values.  
# For a full list of options for the 'CAMERA' variable, 
# see the documentation.
CAMERA=OV7692
#CAMERA=OV5640
# CAMERA=HM0360_MONO
#CAMERA=HM01B0

# Uncomment to use legacy Adafruit 3315 TFT drivers (TFT = ADAFRUIT)
# Otherwise, default drivers (TFT = NEWHAVEN) will be used for NewHaven NHD-2.4
# TFT = ADAFRUIT

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2