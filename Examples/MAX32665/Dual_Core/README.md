## Description

A basic getting started program for using both ARM cores.

Based off the Hello_World example, the console UART, LED toggling, and incremental count are
split between both cores.

Please check the board.c file in ${MSDKPath}\Libraries\Boards\MAX32665\${BoardName}\Source path to learn switch and LED pins for specific board.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

Set `ARM_DUALCORE=1` to build with the Core 1 startup and system files.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32665 Dual Core Example *****
Similar to the 'Hello World' example but split the
lights and console uart between core 0 and core 1.
Halting this example with a debugger will not stop core 1.

Core 1: enter while loop.
Core 1: Ping: 0
Core 0: Pong: 1
Core 1: Ping: 1
Core 0: Pong: 2
Core 1: Pong: 2
```
