# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# Set the camera drivers.  Select a line to match the
# connected camera.  These are some common values.  
# For a full list of options for the 'CAMERA' variable, 
# see the documentation.
CAMERA=OV7692
#CAMERA=OV5642
#CAMERA=HM0360_MONO
#CAMERA=HM01B0

# Set optimization level to -O2, which is required for the CameraIF DMA
# timing to work properly.
MXC_OPTIMIZE_CFLAGS=-O2
