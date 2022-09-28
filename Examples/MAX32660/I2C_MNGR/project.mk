# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default setup isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available options.

# Build the FreeRTOS Library
LIB_FREERTOS = 1

# Tell make file where to find the I2C Manager source files
VPATH += i2c_mngr
IPATH += i2c_mngr