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

# Project config options (see README):
# -----------------------
TFT_ENABLE = 0
# -----------------------

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# This example is only compatible with the FTHR board
BOARD := FTHR_RevA

ifneq "$(BOARD)" "FTHR_RevA"
define ERR_MSG
ERR_NOTSUPPORTED: 
This project is only supported on the MAX78000FTHR (FTHR_RevA)
See https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages
endef
$(error $(ERR_MSG))
endif

# Place build files specific to FTHR_RevA here.
ifeq "$(BOARD)" "FTHR_RevA"
SDHC_CLK_FREQ  = 25000000

ifeq "$(TFT_ENABLE)" "1"
# Only Enable if 2.4" TFT is connected to Feather
PROJ_CFLAGS+=-DTFT_ENABLE
IPATH += TFT/fthr
VPATH += TFT/fthr
FONTS = LiberationSans16x16
endif

endif

LIB_SDHC = 1


