# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add additional project folders to build
IPATH += bootloader
VPATH += bootloader

IPATH += test_images
VPATH += test_images/MAX32660
VPATH += test_images/MAX32670
