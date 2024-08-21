# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add config here

# This example is only compatible with the MAX32655EVKIT
ifneq ($(BOARD),EvKit_V1)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX32655EVKIT.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
endif

# Include MAX31889 drivers from MiscDrivers library.
SRCS+=max31889_driver.c

VPATH+=$(LIBS_DIR)/MiscDrivers/TempSensor
IPATH+=$(LIBS_DIR)/MiscDrivers/TempSensor
