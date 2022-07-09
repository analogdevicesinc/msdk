## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P1.1 is continuously scanned and whatever value is read on that pin is then output to P1.6.  An interrupt is set up on P0.2. P1.7 toggles when that interrupt occurs.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX SEL and TX SEL on headers JP7 and JP8.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED0 EN).
-   Close jumper JP6 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************
1. This example reads P1.1 and outputs the same state onto P1.6.
2. An interrupt is set up on P0.2 when grounded so that P1.7 toggles when that
interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.