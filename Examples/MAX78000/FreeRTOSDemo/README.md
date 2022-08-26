## Description

A basic getting started application for FreeRTOS. 

## Setup

If using the Standard EvKit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-	Select "EvKit_V1" for _BOARD_ in "project.mk"

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-	Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Select "FTHR_RevA" for _BOARD_ in "project.mk"

## Expected Output

The Console UART of the device will output these messages:

```
-=- MAX78000 FreeRTOS (V10.2.0) Demo -=-
SystemCoreClock = 60000000
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
cmd>
```
