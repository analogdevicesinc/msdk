# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# Override the default linkerfile
LINKERFILE=$(TARGET_LC)_spix.ld

ifneq ($(BOARD),EvKit_V1)
$(error This example requires an external flash IC that is only available for the MAX32665EVKIT)
endif
