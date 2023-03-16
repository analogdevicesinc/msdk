# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration
 
#BOARD=FTHR_APPS_A
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_APPS_A" board.

# **********************************************************

# Add your config here!

# If you have secure version of MCU (MAX32651), set SBT=1 to generate signed binary
# For more information on how the signing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Add your static library here
BUILD_DIR = $(CURDIR)/build
LIB_BUILD_DIR = $(BUILD_DIR)/mylib

# Point to the location of the library
LIB_LOCATION = ../TestLib

# Add library to the build
LIB_NAME = mylib
MYLIB = $(LIB_BUILD_DIR)/$(LIB_NAME).a
LIBS += $(MYLIB)

# Add rule to build library with a recursive make call
$(MYLIB):
	make -C $(LIB_LOCATION) BUILD_DIR=$(LIB_BUILD_DIR) PROJECT=$(LIB_NAME) lib