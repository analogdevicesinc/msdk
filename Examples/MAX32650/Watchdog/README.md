## Description

A demonstration of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog.  The buttons on the EV Kit can then be used to prevent the reset of the Watchdog, in order to show the watchdog reset the device..

- Pushing SW2 will put the device in an infinite loop until the watchdog timer expires and the device is reset.
- Pushing SW3 will also put the device in an infinite loop until the watchdog timer expires, however this will additionally enable the watchdog interrupt to be triggered before doing so.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Example ****************
Press a button to trigger a watchdog reset.
SW2 (P2.25) = Reset without interrupt.
SW3 (P2.26) = Reset with interrupt.

Enabling Timeout Interrupt...

TIMEOUT! 
Waiting for watchdog reset...

Watchdog reset

************** Watchdog Timer Example ****************
Press a button to trigger a watchdog reset.
SW2 (P2.25) = Reset without interrupt.
SW3 (P2.26) = Reset with interrupt.

Waiting for watchdog reset...

Watchdog reset

************** Watchdog Timer Example ****************
Press a button to trigger a watchdog reset.
SW2 (P2.25) = Reset without interrupt.
SW3 (P2.26) = Reset with interrupt.
```
