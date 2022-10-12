## Description

A basic getting started application for FreeRTOS. 

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close CTS and RTS jumpers in tickless mode to use as a deep sleep wakeup source. 

## Expected Output

Enable hardware flow control in the terminal to use as a deep sleep wakeup source.

The Console UART of the device will output these messages:

```
-=- 32665 FreeRTOS (V10.2.0) Demo -=-
SystemCoreClock = 96000000
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
cmd>
```
