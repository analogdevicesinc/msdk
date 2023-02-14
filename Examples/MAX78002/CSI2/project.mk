# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Add your config here!

# Set the camera drivers.  Select a line to match the
# connected camera.  These are some common values.  
# For a full list of options for the 'CAMERA' variable, 
# see the documentation.
CAMERA=OV5640

# Set a higher optimization level.  The increased performance
# is required for the Camera DMA code to work within the
# timing requirements of the CSI2 interface.
MXC_OPTIMIZE_CFLAGS=-O2

# Set the CSI2 linkerfile, which reserves an SRAM instance required
# for the CSI2 hardware buffers
LINKERFILE=max78002_csi2.ld

