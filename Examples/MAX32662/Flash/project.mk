# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

LINKERFILE=$(TARGET_LC)_ram.ld
$(info This example executes out of RAM using a special linkerfile: $(LINKERFILE))
PROJ_CFLAGS+=-D__RAM_FWK__
