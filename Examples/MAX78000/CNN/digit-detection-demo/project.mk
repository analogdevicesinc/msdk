# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#BOARD=FTHR_RevA
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_RevA" board.

# **********************************************************

# Add your config here!

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# A higher optimization level is used for this example, but it
# may make debugging unreliable.  Comment out the line below
# to use the default optimization level, which is optimized
# for a good debugging experience.
MXC_OPTIMIZE_CFLAGS = -O2

ifeq "$(BOARD)" "EvKit_V1"
VPATH += TFT/evkit/resources
endif
ifeq "$(BOARD)" "FTHR_RevA"
VPATH += TFT/fthr
endif

IPATH += TFT/evkit/resources

