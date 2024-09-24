# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# Load and start the RISCV core
RISCV_LOAD=1

# Directory for RISCV code
RISCV_APP=../RISCV

IPATH += ./inc
VPATH += ./src

DEV_MODE_TRACE = 1
ifeq ($(DEV_MODE_TRACE), 1)
PROJ_CFLAGS += -DDEV_MODE_TRACE=1
endif

MAILBOX_SIZE = 226

ifeq ($(BOARD),FTHR_Apps_P1)
$(error ERR_NOTSUPPORTED: This example requires a TFT display, therefore it's not supported on the MAX32650FTHR)
endif
