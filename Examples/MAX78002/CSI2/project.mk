# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Add your config here!

# Set OV5640 camera drivers for use with the Pcam 5C.
# These are the only drivers tested with this example.
CAMERA=OV5640

# Add drivers for the APS6404 SRAM that is on-board the
# MAX78002EVKIT.  We use this to buffer the incoming image
# data.
VPATH += src/sram
IPATH += src/sram
