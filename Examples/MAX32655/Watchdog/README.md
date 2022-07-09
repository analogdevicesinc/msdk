## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  The application then begins to reset the watchdog within the allowed window.  Use PB1 and PB2 on the board to control if and when the application attempts to reset the timer.

- Push PB1 to trigger a "too-late" watchdog reset. This will stop resetting the watchdog timer until it generates the "too-late" interrupt.  After that it will reset the watchdog timer only once, allowing it to pass the reset timeout period.
- Push PB2 to reset the watchdog timer in the "too-early" period.

On the Standard EV Kit:
-	PB1: P0.18/SW3
-	PB2: P0.19/SW4

On the Featherboard:
-	PB1: P0.2/SW2
-	PB2: P0.3/SW3

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the board version you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
************** Watchdog Timer Demo ****************
PB1: Push PB1 to trigger a "too-late" watchdog reset. This will stop resetting
     the watchdog timer until it generates the "too-late" interrupt.  After that
     it will reset the watchdog timer only once, allowing it to pass the reset
     timeout period.

PB2: Push PB2 to reset the watchdog timer in the "too-early" period.
```

