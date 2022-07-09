## Description

This example demonstrates the use of the CRC feature of the Trust Protection Unit. 

To demonstrate, CRC values are calculated in software and using the TPU. The results of these are calculations are compared to verify the results. This process is repeated for both 16-bit and 32-bit polynomials. 

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** CRC Example *****
CRC16:
Calculated CRC = 0x0000b16c
Expected CRC   = 0x0000b16c
CRC Passed!

CRC32:
Calculated CRC = 0x3906c3e5
Expected CRC   = 0x3906c3e5
CRC Passed!

Example complete.
```
