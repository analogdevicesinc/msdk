# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
PROJ_CFLAGS+=-DENABLE_TFT
IPATH += TFT/evkit/
VPATH += TFT/evkit/
endif

# If enabled, it sends out the Mic samples used for inference to the serial port
#PROJ_CFLAGS+=-DSEND_MIC_OUT_SERIAL

# Place build files specific to FTHR_RevA here.
ifeq "$(BOARD)" "FTHR_RevA"
# Only Enable if 2.4" TFT is connected to Feather
#PROJ_CFLAGS+=-DENABLE_TFT

# If enabled, it saves out the Mic samples used for inference to SDCARD
# Note that if both SDCARD and TFT are enabled, the TFT will be disabled to avoid SPI driver conflict.
#PROJ_CFLAGS+=-DSEND_MIC_OUT_SDCARD

# If enabled, it captures audio from line input of MAX9867 audio codec instead of the on-board mic.
# Note that SEND_MIC_OUT_SDCARD should be disabled in this mode
#PROJ_CFLAGS+=-DENABLE_CODEC_MIC
LIB_SDHC = 1
IPATH += TFT/fthr
VPATH += TFT/fthr
endif



