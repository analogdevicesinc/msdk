## Description

This example demonstrates various functions of the MAX32680's flash controller, including flash mass erase, flash page erase and wiriting data to flash.

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```

***** Flash Control Example *****
Flash erased.
Flash mass erase is verified.
Writing 8192 32-bit words to flash
Size of testdata : 32768
Word 0 written properly and has been verified.
Word 1 written properly and has been verified.
Word 2 written properly and has been verified.
Word 3 written properly and has been verified.
Word 4 written properly and has been verified.
Word 5 written properly and has been verified.
Word 6 written properly and has been verified.
Word 7 written properly and has been verified.
Word 8 written properly and has been verified.
Word 9 written properly and has been verified.
Word 10 written properly and has been verified.
Word 11 written properly and has been verified.
Word 12 written properly and has been verified.
Word 13 written properly and has been verified.
Word 14 written properly and has been verified.
Word 15 written properly and has been verified.
Continuing for 8176 more words...
Page Erase is verified
Flash Erase is verified

Example Succeeded
