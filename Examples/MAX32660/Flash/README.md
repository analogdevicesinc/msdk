## Description

TBD<!--TBD-->

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Flash Control Example *****
Flash erased.
Flash mass erase is verified.
Writing 2048 128-bit words to flash
Size of testdata : 32768
Word 0 written properly and has been verified.
Word 1 written properly and has been verified.
Word 2 written properly and has been verified.
Word 3 written properly and has been verified.
Continuing for 2044 more words...
Page Erase is verified
Flash Erase is verified

Example Succeeded
```
