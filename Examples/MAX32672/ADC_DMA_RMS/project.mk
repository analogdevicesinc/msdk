# This file can be used to set build configuration
# variables.  These variables are defined in a file called
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

MXC_OPTIMIZE_CFLAGS += -O3

# Flags to control the different output methods for RMS values+
PROJ_CFLAGS += -DCONSOLE_OUTPUT
#PROJ_CFLAGS += -DAD5592_OUTPUT