## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog.  Use SW2 on the evaluation kit to control if and when the application attempts to reset the timer.

- Hold down SW2 to trigger a "too-late" watchdog reset. This will stop resetting the watchdog timer until it generates the "too-late" interrupt.  After that it will reset the watchdog timer only once, allowing it to pass the reset timeout period.

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. Until the
button is pressed, the watchdog will be continually reset
in the main loop.

Pressing button (SW2) will prevent the watchdog from
reseting, allowing the timer to expire and the device to reset.

Watchdog configured.
Press (and hold) SW2 to allow Watchdog to expire.
```

