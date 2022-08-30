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

# Set the camera drivers.  Select a line to match the
# connected camera.  These are some common values.  
# For a full list of options for the 'CAMERA' variable, 
# see the documentation.
CAMERA=OV7692
#CAMERA=OV5640
#CAMERA=HM0360_MONO
#CAMERA=HM01B0

LIB_SDHC = 1
