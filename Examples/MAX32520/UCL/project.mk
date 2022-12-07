# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#BOARD=MAX32520FTHR
# ^ For example, you can uncomment this line to make the 
# project build for the "MAX32520FTHR" board.

# **********************************************************

# Enable UCL library
LIB_UCL = 1
export UCL_VERSION=2.7.0

# Add project's include and source paths
VPATH += ./src
VPATH += ./src/cipher
VPATH += ./src/public_key
VPATH += ./src/mac

IPATH += ./src/include
