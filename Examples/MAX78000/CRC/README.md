## Description

This example demonstrates the use of the HW CRC calculator.  The example first generates a 32-bit CRC over 4096 bytes.  The resulting CRC is inverted and placed at the end of the 4096+4 bytes.  Another CRC is then calculated over the resulting 4096+4 bytes.  The final result is compared with the expected value.  This sequence is repeated twice - once using the blocking CRC functions and once using the non-blocking (asynchronous) functions.

## Setup
##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:
If using the standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
CRC Sync and Async Example

TEST CRC SYNC

Computed CRC: 13b5a483
CRC Check Result: debb20e3
**Test Passed**

TEST CRC ASYNC

Computed CRC: 13b5a483
CRC Check Result: debb20e3
**Test Passed**


Example Succeeded
```
