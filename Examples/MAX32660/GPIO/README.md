## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P0.11 is continuously scanned and whatever value is read on that pin is then output to P0.13 (LED).  An interrupt is set up on P0.12 (SW2). P0.10 toggles when that interrupt occurs.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.11 (IN) to P0.13 (OUT).

## Expected Output

The Console UART of the device will output these messages:

```
****** GPIO Example ******

1. For this example please connect pins P0.13 and P0.11. The output
   from pin P0.13 is toggled every half second, which is evident from
   both the blinking LED and the value read on pin P0.11.
2. An interrupt is set to occur when SW2 is pressed (P0.12). In the
   ISR P0.10 is toggled for every switch
```

You will also observe the LED behavior given in the Description section above.