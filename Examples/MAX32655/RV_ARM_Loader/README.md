## Description

A basic getting started program for the RISCV, run from the ARM core.

RV_ARM_Loader runs on the ARM core to load the RISCV code space, setup the RISCV debugger pins, and start the RISCV core.

The Hello_World example runs on the the RISCV core. 

## Setup

##### Linker File Selection
Before building firmware you must select the correct linker file (line 111) in the "project.mk" to in the Hello_World example. The Hello_World example must be in the same workspace as the the RV_ARM_Loader example.

```
LINKER=$(TARGET_LC)_riscv.ld
```

## Expected Output

The Console UART of the device will output these messages:

```
Hello World!
count : 0
count : 1
count : 2
count : 3
```

