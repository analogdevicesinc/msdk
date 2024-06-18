# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# TrustZone project with secure and non-secure code.
TRUSTZONE=1

# This is a secure project.
MSECURITY_MODE=SECURE

# Add path to Non-Secure project.
NONSECURE_CODE_DIR=../NonSecure
SECURE_CODE_DIR=.
