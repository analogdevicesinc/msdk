## Description

Basic GPIO input, output, and interrupts are demonstrated in this example.

P2.11 (P1.11 on MAX32690FTHR) is continuously scanned and whatever value is read on that pin is then output to LED 1.  An interrupt is set up on P2.11, LED 2 is toggled on each falling edge.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

Interrupt feature is only shown on MAX32690EVKIT because input buttons cannot be shorted to P1.11 on MAX32690FTHR.

## Required Connections

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pins P4.0 (on header JH5) and P2.11 (on header JH4).
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2_EN).

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pin P1.11 (on header J1, 5th pin) and ground (on header J2, 4th pin).

## Expected Output

The Console UART of the device will output these messages (pin numbers will may be different depending on which board the example is built for):

```
************************* GPIO Example ***********************

1. This example reads P2.11 and outputs the same state onto P0.14.
2. A falling edge interrupt is set up on P2.11. P2.12 toggles when that interrupt occurs.

Connect P4.0->P2.12 to use SW2 to trigger a falling edge interrupt on each press.
```

You will also observe the LED behavior given in the Description section above.
