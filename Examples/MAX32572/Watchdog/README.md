## Description

A demonstration of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog.  Use PB1 on the evaluation kit to control if and when the application attempts to reset the timer.

- Push PB1 to trigger the watchdog reset. This will stop resetting the watchdog timer until it generates the interrupt.  After that it will reset the watchdog timer only once, allowing it to pass the reset timeout period.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
Press a button to create watchdog interrupt or reset:
SW2 (P0.16)= reset program
SW3 (P0.17)= timeout interrupt

Enabling Timeout Interrupt...

TIMEOUT!

Watchdog reset
```
