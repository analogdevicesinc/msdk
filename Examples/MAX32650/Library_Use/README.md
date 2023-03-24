## Description

Simple program that demonstrates how to link static libraries to an application.

This demo application toggles an LED using GPIO functions found in the static library myLib.a. Refer
to project.mk to see how to include ".a" static library files into your project.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This example supports all available MAX32650 evaluation platforms but comes _pre-configured_ for the MAX32650EVKIT by default. See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) for instructions on how to configure the project for a different board.

## Required Connections

If using the MAX32650EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB TO UART) connector.
-   Connect a debugger to J3 or J4 (JTAG/SWD).
-   Select RX SEL and TX SEL on headers JP12 and JP13.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX32650FTHR:
-   Connect a USB cable between the PC and the J1 connector.
-   Connect a debugger to J5.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************** Static Library Example **********************

This example calls static library functions to toggle an LED.

...
```
