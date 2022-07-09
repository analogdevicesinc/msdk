## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog within the allowed window.  Use SW2 when the application attempts to reset the timer.

- If "OVERFLOW" is defined, pressing SW2 will intentionally stall the micro until the reset window has passed, triggering a "too-late" watchdog reset.
- If "UNDERFLOW is defined, pressing SW2 will reset the watchdog timer before the reset window, triggering a "too-early" watchdog reset.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. You can
select between two tests: Timer Overflow and Underflow.

Press a button to create watchdog interrupt and reset:
SW2 (P4.0) = timeout and reset program


Enabling Timeout Interrupt...

TIMEOUT!

Watchdog Reset occured too soon (UNDERFLOW)
```

