## Description

This example demonstrates the use of the HW CRC calculator.  The example first generates a CRC over 100 bytes.  The resulting CRC is inverted and placed at the end of the 100 bytes.  Another CRC is then calculated over the resulting 101 bytes.  The final result is compared with the expected value.  This sequence is repeated twice - once using the blocking CRC functions and once using the non-blocking (asynchronous) functions.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).

## Expected Output

The Console UART of the device will output these messages:

```
************ CRC Example ***********
Test CRC Sync
   Passed

Test CRC Async
   Passed
```


