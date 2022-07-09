## Description

A basic getting started application for FreeRTOS. 

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP2 (LED0 EN).
-   Close jumper JP3 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
-=- MAX32680 FreeRTOS (V10.2.0) Demo -=-
SystemCoreClock = 100000000
Starting scheduler.
Uptime is 0x00000000 (0 seconds), tickless-idle is disabled

Enter 'help' to view a list of available commands.
cmd>
```
