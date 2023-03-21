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

CAMERA=OV5640

VPATH += src/sram
IPATH += src/sram

# Set a higher optimization level.  The increased performance
# is required for the Camera DMA code to work within the
# timing requirements of the CSI2 interface.
# MXC_OPTIMIZE_CFLAGS=-Og

# Set the CSI2 linkerfile, which reserves an SRAM instance required
# for the CSI2 hardware buffers
LINKERFILE=max78002_csi2.ld

