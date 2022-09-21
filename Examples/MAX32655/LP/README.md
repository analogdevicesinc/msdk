## Description

TBD<!--TBD-->

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the board version you are using to run the example.

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
****Low Power Mode Example****

This code cycles through the MAX32655 power modes, using a push button (PB1) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering DEEPSLEEP mode.
Waking up from DEEPSLEEP mode.

```
