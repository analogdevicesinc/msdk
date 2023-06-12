# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

CAMERA = OV5640

VPATH += src/sram
IPATH += src/sram

VPATH += src/tft
IPATH += src/tft

VPATH += src/camera
IPATH += src/camera

VPATH += src/cnn
IPATH += src/cnn
