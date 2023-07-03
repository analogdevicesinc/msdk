# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#BOARD=FTHR_RevA
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_RevA" board.

# **********************************************************

# Add your config here!

# Enable Secure Boot Tool integration
SBT=1

# Define die revision for SBT files
PROJ_CFLAGS+=-DMAX32651_A4

# Set default goal to scpa.  This means that running just 'make'
# is equivalent to 'make scpa'
override .DEFAULT_GOAL=scpa

# Force the project output filename to match the one that's expected
# by the "scp_script.txt" file.
override PROJECT=SCPA_OTP_Dump