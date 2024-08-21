## Description

Simple program that demonstrates how to link static libraries to an application.

This demo application toggles an LED using GPIO functions found in the static library myLib.a. Refer
to project.mk to see how to include ".a" static library files into your project.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This example supports all available MAX78000 evaluation platforms but comes _pre-configured_ for the MAX78000EVKIT by default. See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) for instructions on how to configure the project for a different board.

## Required Connections

If using the MAX78000EVKIT:
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Select "EvKit_V1" for BOARD in "project.mk"

If using the MAX78000FTHR:
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************** Static Library Example **********************

This example calls static library functions to toggle an LED.

...
```
