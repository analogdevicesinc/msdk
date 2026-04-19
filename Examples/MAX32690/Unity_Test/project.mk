# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
LIB_UNITY = 1

# Add 'simple_code.c' to the compilation list for any host-side tests run on `make test`
# Everything else in the 'test' folder will get added automatically.
TEST_SRCS += simple_code.c
