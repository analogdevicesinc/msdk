## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P0.16 is continuously scanned and whatever value is read on that pin is then output to P0.17.  An interrupt is set up on P0.21 (PB1). P0.22 (LED1) toggles when that interrupt occurs.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads P0.16 and outputs the same state onto P0.17.
2. An interrupt is set up on P0.21 . P0.22 toggles when that
   interrupt occurs.
```