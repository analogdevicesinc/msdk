# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system



# **********************************************************

# Add your config here!

# Define the EXT_CLK frequency.  This overrides the default 12.5Mhz
# value set in the system header file.
PROJ_CFLAGS += -DEXTCLK_FREQ=2000000
