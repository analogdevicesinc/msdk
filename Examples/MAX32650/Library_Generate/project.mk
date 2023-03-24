# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system
 
# **********************************************************

# Add your config here!

# If you have secure version of MCU (MAX32651), set SBT=1 to generate signed binary
# For more information on how the signing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Generate a static library by default
override .DEFAULT_GOAL = lib
