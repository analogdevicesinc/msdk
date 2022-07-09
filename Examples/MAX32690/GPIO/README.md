## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P2.11 is continuously scanned and whatever value is read on that pin is then output to LED 1.  An interrupt is set up on P2.11, LED 2 is toggled on each falling edge.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Connect pins P4.0 (on header JH5) and P2.11 (on header JH4).
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2_EN).

## Expected Output

The Console UART of the device will output these messages (pin numbers will may be different depending on which board the example is built for):

```
************************* GPIO Example ***********************

1. This example reads P2.11 and outputs the same state onto P0.14.
2. A falling edge interrupt is set up on P2.11. P2.12 toggles when that interrupt occurs.

Connect P4.0->P2.12 to use SW2 to trigger a falling edge interrupt on each press.
```

You will also observe the LED behavior given in the Description section above.