## Description

A basic getting started application for FreeRTOS. 

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
-=- MAX32670 FreeRTOS (V10.2.0) Demo -=-
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
cmd> 
```
