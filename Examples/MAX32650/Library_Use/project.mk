# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration
 
BOARD=FTHR_APPS_A
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_APPS_A" board.

# **********************************************************

# Add your config here!

# If you have secure version of MCU (MAX32651), set SBT=1 to generate signed binary
# For more information on how the signing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Point to the location of the library
LIB_LOCATION = ../Library_Generate
# Add the location of the library to the list of include paths
IPATH += $(LIB_LOCATION)
 
# Add library to the list of dependencies
LIB_NAME = Library_Generate
LIB_BUILD_DIR = $(LIB_LOCATION)/build
MYLIB = $(LIB_BUILD_DIR)/$(LIB_NAME).a
LIBS += $(MYLIB)
