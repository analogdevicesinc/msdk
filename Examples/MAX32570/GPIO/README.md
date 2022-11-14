## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

If your board QN_EvKit or Q_EvKit
   - PB1(P3.7) is continuously scanned and whatever value is read on that pin is then output to LED1(P3.5).  
   - An interrupt is set up on PB2(P3.6)  LED2(P3.4) toggles when that interrupt occurs.

If your board MN_EvKit or M_EvKit
   - PB1(P0.16) is continuously scanned and whatever value is read on that pin is then output to LED1(P2.17).  

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads PB1(P3.7) and outputs the same state onto LED1(P3.5).
2. An interrupt is set up on PB2(P3.6)  LED2(P3.4) toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.