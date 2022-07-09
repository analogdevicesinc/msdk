## Description

This example uses the various functions of the True Random Number Generator. Specifically, it shows the acquisition of both 32-bit and 128-bit numbers, doing so using both synchronous and asynchronous acquistions.   

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** TRNG Example *****

4 Random 32 Bit Integers
0xab32a49a
0x8441b423
0x2255b583
0xf7873e64

Synchronously Acquired Random 128-Bit Number
0-3: 0x5d7838b5
4-7: 0x70a13fed
8-b: 0xd9783a23
c-f: 0x8a7d54f8

Asynchronusly Acquired Random 128-Bit Number
0-3: 0xb6326b12
4-7: 0x72d628ac
8-b: 0xd00851e7
c-f: 0xadf7981b

Example complete.
```
