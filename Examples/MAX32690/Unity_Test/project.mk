# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
LIB_UNITY=1

TEST_SRCS+=simple_code.c
DEBUG=1
PROJ_CFLAGS += -DUNITY_INCLUDE_CONFIG_H
# TEST_CFLAGS+=-DUNITY_INCLUDE_CONFIG_H
# TEST_CC=gcc
# TEST_CFLAGS=-Werror
