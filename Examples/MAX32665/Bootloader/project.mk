# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# If you have secure version of MCU (MAX32666), set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Optimize for size
MXC_OPTIMIZE_CFLAGS=-Os

# Strip debug symbols
DEBUG=0

# To close display related code in board.c file
PROJ_CFLAGS+=-DNO_DISPLAY

SRCS += boot_lower.S
LINKERFILE = bootloader.ld

AUTOSEARCH=0
USE_INTERNAL_FLASH ?=0
ifeq ($(USE_INTERNAL_FLASH), 1)
SRCS += main_int.c
else
SRCS += main_ext.c
endif