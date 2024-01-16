# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!


SRCS+=eeprom_24lc256_driver.c

VPATH+=$(LIBS_DIR)/MiscDrivers/EEPROM
IPATH+=$(LIBS_DIR)/MiscDrivers/EEPROM
