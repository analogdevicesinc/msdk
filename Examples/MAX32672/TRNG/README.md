## Description

The true random number generator (TRNG) hardware is exercised in this example.  Random values are generated both using the blocking and non-blocking (asynchronous) functions.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********** TRNG Example **********

Test TRNG Sync
0x40 0x78 0xdf 0xc7
0x08 0xe7 0xfb 0x1c
0x60 0xf8 0xdf 0xc3
0x00 0xe7 0x7f 0x18

Test TRNG Async
0x08 0xc6 0xf7 0x39
0xc7 0x00 0x18 0xdf
0x9f 0x83 0x20 0xbc
0xfc 0x1f 0x03 0xf0

********** Test Complete **********
```