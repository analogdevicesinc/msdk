## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P2.28 (SW2) is continuously scanned and whatever value is read on that pin is then output to P2.25 (LED0). An interrupt is set up on P2.30 (SW3). P2.26 (LED1) toggles when that interrupt occurs.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****** GPIO Example ******

1. This example reads P2.28 (SW2) and outputs the same state onto P2.25 (LED0).
2. An interrupt is set up on P2.30 (SW3). P2.26 (LED1) toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.