# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# Build with the necessary Core1 startup/system files.
ARM_DUALCORE=1

# Separate directories for Core 0 and Core 1 code.
VPATH += Core0
VPATH += Core1

IPATH += Core0
IPATH += Core1
