# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

# Build this project for the RISC-V core
# Note: This project is typically not compiled on its own.
# Use ../Dual_core_sync_arm, which will build this project for RISC-V and package it
# alongside the Arm code in the same binary.
RISCV_CORE=1
