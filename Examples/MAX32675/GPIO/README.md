## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

PB0 is continuously scanned and whatever value is read on that pin is then output to LED0.  An interrupt is set up on PB1. LED1 toggles when that interrupt occurs.

Please check the board.c file in ${MSDKPath}\Libraries\Boards\MAX32675\${BoardName}\Source path to learn push button and LED pins for specific board.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   For EvKit close
     -   Jumper JP1 (LED0 EN).
     -   Jumper JP2 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
************************* GPIO Example ***********************

1. This example reads P1.11 (SW1) and outputs the same state onto P1.09 (LED0).
2. An interrupt is set up on P1.12 (SW2). P1.10 (LED1) toggles when that
   interrupt occurs.
```