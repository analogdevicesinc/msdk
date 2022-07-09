## Description

This example is the introduction to using the RISC-V core.

The RISC-V core runs the same code as is found in the Hello_World example, with the addition of initializing the RISC-V pins and enabling the RISC-V core's instruction cache.

The ARM core initializes the RISC-V core before relinquishing control of execution to it.

## Setup

##### Building Firmware:

Before building firmware you must select the correct value for _BOARD_  in Makefile.ARM and Makefile.RISCV, either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

***** Note *****: Be sure to use the max78000-combined.elf executable in the _Build_ directory.

##### Required Connections:

If using the standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
Waiting...
Hello World!
count = 0
count = 1
count = 2
count = 3
count = 4
count = 5
...
```

