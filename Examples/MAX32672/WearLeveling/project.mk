# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

LIB_LITTLEFS = 1

# This example attempts to mount a Little FS filesystem on half of the
# device's Flash space. To ensure application code does not use any of
# the Flash space that Little FS will attempt to use, the available 
# Flash size is reduced with one of the custom linkerfiles below.
ifeq ($(SBT),1)
# This linkerfile contains extra sections for compatibility with the Secure Boot Tools (SBT).
override LINKERFILE = wearlevel-sla.ld
else
# This linkerfile is for use with standard non-secure applications.
override LINKERFILE = wearlevel.ld
endif # SBT
