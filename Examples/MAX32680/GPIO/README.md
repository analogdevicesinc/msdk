## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

PB1 is continuously scanned and whatever value is read on that pin is then output to LED 1.  An interrupt is set up on button 2, LED 2 is toggled each time that button is pressed.

On the Standard EV Kit:
-	PB1: P0.18/SW3
-	PB2: P0.19/SW4
-	LED 1: P0.24/LED0
-	LED 2: P0.25/LED1

On the Featherboard:
-	PB1: P0.2/SW2
-	PB2: P0.3/SW3
-	LED 1: P0.18/Red LED
-	LED 2: P0.26/Blue LED

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the EV kit you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP2 (LED0 EN).
-   Close jumper JP3 (LED1 EN).

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages (pin numbers will may be different depending on which board the example is built for):

```
************************* GPIO Example ***********************

1. This example reads P0.18 and outputs the same state onto P0.24.
2. An interrupt is set up on P0.19. P0.25 toggles when that
   interrupt occurs.
```

You will also observe the LED behavior given in the Description section above.