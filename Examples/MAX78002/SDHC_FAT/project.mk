# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Add your config here!

# Enable CLI library
LIB_CLI = 1
# Enable SDHC library
LIB_SDHC = 1
# Use FatFS R0.15
FATFS_VERSION = ff15
# Set 30Mhz SDHC clock frequency since MAX78002 initializes with 60Mhz system clock by default.
# SDHC_CLK_FREQ can be increased to 60Mhz if the system clock is switched to the 120Mhz IPO.
SDHC_CLK_FREQ = 30000000

