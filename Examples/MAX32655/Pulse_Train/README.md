## Description

Two pulse trains are configured in different modes.  

The first, PT0/PT2, is set to generate a repeating bit pattern of 0x10110 (lsb first) at a rate of 2 bits per second.  If you make the connections described below, you can observe the pattern on LED 1.

The second, PT1/PT3, is set to generate a 10Hz square wave.  If you make the connections described below, you can observe the square wave on LED 2.

On the standard EV Kit:
-    PT2: P0.16
-    PT3: P0.17

On the Featherboard:
-    PT0: P0.18/Red LED
-    PT1: P0.19/Green LED

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the board version you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pin 16 of JH7 (GPIO 0 Port) to pin 1 of JP2 (LED0).
-   Connect pin 17 of JH7 (GPIO 0 Port) to pin 1 of JP3 (LED1).

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************** Pulse Train Demo ****************
PT2 (P0.16) = Outputs continuous pattern of 10110b at 2bps
PT3 (P0.17) = Outputs 10Hz continuous square wave
```

