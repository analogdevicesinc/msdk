## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog within the allowed window.  Use SW2 and SW3 on the evaluation kit to control if and when the application attempts to reset the timer.

- Push SW2 to trigger a "too-late" watchdog reset. This will stop resetting the watchdog timer until it generates the "too-late" interrupt.  After that it will reset the watchdog timer only once, allowing it to pass the reset timeout period.
- Push SW3 to reset the watchdog timer in the "too-early" period.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

- For a "quick-start" or for first-time users see ["Getting Started"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#getting-started)
- See ["Development Guide"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#development-guide) for a detailed reference.

### Project-Specific Build Notes

(None)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
SW2: Push SW2 to trigger a "too-late" watchdog reset. This will stop resetting
     the watchdog timer until it generates the "too-late" interrupt.  After that
     it will reset the watchdog timer only once, allowing it to pass the reset
     timeout period.

SW3: Push SW3 to reset the watchdog timer in the "too-early" period.
```

