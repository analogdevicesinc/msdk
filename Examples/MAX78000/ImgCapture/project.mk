# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Project options:
# Set to 1 to enable, 0 to disable
CONSOLE = 1
SD = 0

# Set the camera drivers.  Select a line to match the
# connected camera.  These are some common values.  
# For a full list of options for the 'CAMERA' variable, 
# see the documentation.
CAMERA=OV7692
# CAMERA=OV5642
# CAMERA=HM0360_MONO
# CAMERA=HM0360_COLOR
# CAMERA=HM01B0

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS=-O2

ifeq ($(CONSOLE),1)
# If CONSOLE enabled, add "CONSOLE"
PROJ_CFLAGS += -DCONSOLE
VPATH += src/console
endif

ifeq ($(SD),1)
# If SD enabled, add "SD" compiler definition,
# enable SD card library, and add src/sd to the
# build.
PROJ_CFLAGS += -DSD
LIB_SDHC = 1
VPATH += src/sd
endif
