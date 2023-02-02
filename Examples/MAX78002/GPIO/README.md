## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P2.6 (PB1) is continuously scanned and whatever value is read on that pin is then output to P2.4 (LED1).  An interrupt is set up on P2.7 (PB2). P2.5 (LED2) toggles when that interrupt occurs.

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
-   Close jumper JP47 (LED0 EN).
-   Close jumper JP48 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads P2.6 and outputs the same state onto P2.4.
2. An interrupt is set up on P2.7. P2.5 toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.