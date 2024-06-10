# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
LIB_UNITY=1

TEST_SRC+=test/test_runner.c
TEST_SRC+=test/test_functions.c
TEST_SRC+=simple_code.c


# TEST_CC=gcc
# TEST_TARGET_OUT=build/testbench
# TEST_CFLAGS=-Werror
