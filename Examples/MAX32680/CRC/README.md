## Description

This example demonstrates the use of the HW CRC calculator.  The example first generates a CRC over 100 bytes.  The resulting CRC is inverted and placed at the end of the 100 bytes.  Another CRC is then calculated over the resulting 101 bytes.  The final result is compared with the expected value.  This sequence is repeated twice - once using the blocking CRC functions and once using the non-blocking (asynchronous) functions.

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

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


