## Description

Simple program that demonstrates how to build and link static libraries.

The demo application does uppercase and lowercase string conversion using functions provided by an
example static library.

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
******************Static Library Example****************

This example continuously does uppercase and lowercase
string conversions in a loop using static library calls.
The results are then printed to the console.



HELLO WORLD
hello world
HELLO WORLD
hello world
HELLO WORLD
hello world
...
```

