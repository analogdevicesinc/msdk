# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Set SDMA size by passing a flag to the assembler
PROJ_AFLAGS+=-D__MSR_SDMA_SIZE=0xA000
