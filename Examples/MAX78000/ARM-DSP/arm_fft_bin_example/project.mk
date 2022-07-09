# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default project configuration isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available config variables.

# Include the CMSIS-DSP library
MFLOAT_ABI = hard
include $(CMSIS_ROOT)/5.9.0/DSP/CMSIS-DSP.mk 

