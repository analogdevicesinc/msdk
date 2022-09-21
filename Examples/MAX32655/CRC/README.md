## Description

This example demonstrates the use of the HW CRC calculator.  The example first generates a CRC over 100 bytes.  The resulting CRC is inverted and placed at the end of the 100 bytes.  Another CRC is then calculated over the resulting 101 bytes.  The final result is compared with the expected value.  This sequence is repeated twice - once using the blocking CRC functions and once using the non-blocking (asynchronous) functions.

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the EV kit you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
CRC Sync and Async Example

TEST CRC SYNC

CRC Poly Result: 13b5a483
CRC Check Result: debb20e3
**Test Passed**

TEST CRC ASYNC

CRC Poly Result: 13b5a483
CRC Check Result: debb20e3
**Test Passed**


Example Succeeded
```


