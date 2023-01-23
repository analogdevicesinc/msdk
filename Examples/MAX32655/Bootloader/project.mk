# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Optimize for size
MXC_OPTIMIZE_CFLAGS=-Os

# Strip debug symbols
DEBUG=0

BUILD_DIR:=./build

SRCS += boot_lower.S
LINKERFILE = bootloader.ld

AUTOSEARCH=0
USE_INTERNAL_FLASH ?=0
ifeq ($(USE_INTERNAL_FLASH), 1)
SRCS += main_int.c
else
SRCS += main_ext.c
endif