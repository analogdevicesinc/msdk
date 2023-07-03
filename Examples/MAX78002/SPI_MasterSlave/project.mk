# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# SPI v2 does not support Target (L. Slave) Transaction functions yet.
# Set the MXC_SPI_BUILD_LEGACY to 1 to build the previous SPI library.
MXC_SPI_BUILD_LEGACY=1
