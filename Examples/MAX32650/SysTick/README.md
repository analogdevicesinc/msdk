## Description

This example demostrates the use of the SysTick Interrupt on the MAX32650. The systick interrupt is configured to fire every 40ms and in each interrupt handler LED0 is toggled.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************ Blinky SysTick ****************
SysTick Clock = 120000000 Hz
SysTick Period = 4800000 ticks
```
