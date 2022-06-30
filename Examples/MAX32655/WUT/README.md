## Description
This Example shows how to wake up a device after it is asleep with a wake up timer.  After a defined number of seconds it will wake up after going to sleep.

On the standard EV Kit:
-    PB0: P0.18/SW3

On the Featherboard:
-    PB0: P0.2/SW2

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

```
/************** Wakeup timer example ********************/
This example is to show how the Wakeup timer is used and configured
Press PB0 to put the chip into sleep and then the wakeup timer will wake up in 5000 Miliseconds
Entering SLEEP mode.
Waking up from SLEEP mode.
```
