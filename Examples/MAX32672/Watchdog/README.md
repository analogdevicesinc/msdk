## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer. The application then begins to reset the watchdog within the allowed window. 

When SW3 (P0.18) is pressed, the application will intentionally force a watchdog reset. If "UNDERFLOW" is defined at the top of *main*, the WDT count reset will occur before the window, causing a "too soon" WDT system reset. If "OVERFLOW" is defined at the top of *main*, then the device will wait in an infinite loop until the window passes, causing a "too late" WDT system reset.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

LED0 will flash quickly several times after reset occurs, then will blink at a rate of 1Hz until SW3 is pressed. 

The Console UART of the device will output these messages:

If "OVERFLOW" is defined:

```
******************** Watchdog Timer Demo ********************
This example demonstrates the WDT in windowed mode. With UNDERFLOW
defined the WDT count reset will occur before the window, causing
a "too soon" WDT system reset. With OVERFLOW defined the device
will wait in an infinite loop until the window expires, causing a
"too late" WDT system reset

Press push button SW3 (P0.18) to trigger the WDT interrupt and system
reset described above.

Watchdog reset window configured.
Starving the dog until reset window expires...

TIMEOUT!

Watchdog Reset occurred too late (OVERFLOW)
```

If "UNDERFLOW" is defined:

```
******************** Watchdog Timer Demo ********************
This example demonstrates the WDT in windowed mode. With UNDERFLOW
defined the WDT count reset will occur before the window, causing
a "too soon" WDT system reset. With OVERFLOW defined the device
will wait in an infinite loop until the window expires, causing a
"too late" WDT system reset

Press push button SW3 (P0.18) to trigger the WDT interrupt and system
reset described above.

Watchdog reset window configured.
Feeding the dog before entering reset window...

Watchdog Reset occurred too soon (UNDERFLOW)
```

