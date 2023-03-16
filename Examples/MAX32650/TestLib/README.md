## Description

Simple program that demonstrates that demonstrates GPIO functionality.

The demo application reads the push button inputs and toggles the LEDs based on the input states. If you
wish to build this project as a static library, uncomment the "#override .DEFAULT_GOAL = lib" line in 
the project.mk file. You may also use this project as an external library for another project. See
StaticLibrary_Example project as an example.

## Required Connections

### MAX32650 Evaluation Kit
-   Connect a USB cable between the PC and the CN2 (USB TO UART) connector.
-   Connect a debugger to J3 or J4 (JTAG/SWD).
-   Select RX SEL and TX SEL on headers JP12 and JP13.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

### MAX32650FTHR Evaluation Kit
-   Connect a USB cable between the PC and the J1 connector.
-   Connect a debugger to J5.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****** GPIO Library Example ******

This example reads the state of SW1 and SW2 buttons then toggles
RED and GREEN LEDs according to the state of the push buttons.
This project can also be compiled as a static library that can be
linked to another application.
...
```

