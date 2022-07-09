## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P0.16 is continuously scanned and whatever value is read on that pin is then output to P0.17.  An interrupt is set up on P0.18. P0.19 toggles when that interrupt occurs.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads P0.16 and outputs the same state onto P0.17.
2. An interrupt is set up on P0.18 . P0.19 toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.