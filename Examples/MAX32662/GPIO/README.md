## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P0.04 is continuously scanned and whatever value is read on that pin is then output to P0.5.  An interrupt is set up on P0.6 (SW2). P0.14 (LED) toggles when that interrupt occurs.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads P0.4 and outputs the same state onto P0.5.
2. An interrupt is set up on P0.6. P0.14 toggles when that
   interrupt occurs.

```

You will also observe the LED behavior given in the Description section above.