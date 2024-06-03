# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
LIB_UNITY=1


TEST_CC=gcc
TEST_CFLAGS=-std=c89
TEST_CFLAGS += -Wall
TEST_CFLAGS += -Wextra
TEST_CFLAGS += -Wpointer-arith
TEST_CFLAGS += -Wcast-align
TEST_CFLAGS += -Wwrite-strings
TEST_CFLAGS += -Wswitch-default
TEST_CFLAGS += -Wunreachable-code
TEST_CFLAGS += -Winit-self
TEST_CFLAGS += -Wmissing-field-initializers
TEST_CFLAGS += -Wno-unknown-pragmas
TEST_CFLAGS += -Wstrict-prototypes
TEST_CFLAGS += -Wundef
TEST_CFLAGS += -Wold-style-definition


TARGET_OUT=testbench
UNITY_DIR=$(LIBS_DIR)/Unity
TEST_SRC=$(UNITY_DIR)/src/unity.c
TEST_SRC+=test/test_runner.c
TEST_SRC+=test/test_functions.c

TEST_SRC+=simple_code.c

TEST_INC += -I$(LIBS_DIR)/Unity/src

unit:
	$(TEST_CC) $(TEST_INC) $(TEST_CFLAGS) $(TEST_SRC) -o $(TARGET_OUT) 
	- ./$(TARGET_OUT)
