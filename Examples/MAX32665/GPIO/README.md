## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

Push button 0 is continuously scanned and whatever value is read on that pin is then output to LED0.  An interrupt is set up on Push button 1. LED1 toggles when that interrupt occurs.

Please check the board.c file in ${MSDKPath}\Libraries\Boards\MAX32665\${BoardName}\Source path to learn push button and LED pins for specific board.


## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****** GPIO Example ******

1. This example reads P1.06 (SW2) and outputs the same state onto P1.14 (LED0).
2. An interrupt is set up on P1.07 (SW3). P1.15 (LED1) toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.