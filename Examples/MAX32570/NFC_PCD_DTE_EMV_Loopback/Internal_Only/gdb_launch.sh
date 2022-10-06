#!/bin/sh 

arm-none-eabi-gdb --command=.gdbinit build/*.elf
