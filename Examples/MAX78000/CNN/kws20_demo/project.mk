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

# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
IPATH += TFT/evkit/
VPATH += TFT/evkit/
SRCS += all_imgs.c
endif

# Place build files specific to FTHR_RevA here.
ifeq "$(BOARD)" "FTHR_RevA"
IPATH += TFT/fthr
VPATH += TFT/fthr
SRCS += img_1_rgb565.c
SRCS += logo_rgb565.c
SRCS += SansSerif16x16.c
endif



