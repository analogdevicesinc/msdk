## Description

This example demonstrates the basic functions of the Flash Controller: mass erase, page erase, and write. Initially the flash is cleared using the mass erase function. Next, a page in each flash bank is filled with data and is subsequently cleared using page erase.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Flash Control Example *****
Flash erased.
Flash mass erase is verified.

Writing 8192 32-bit words to flash 0
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

Writing 8192 32-bit words to flash 1
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

Attempting to erase page 0 in flash bank 0
Page Erase is verified

Attempting to erase page 0 in flash bank 1
Page Erase is verified

Attempting to partially erase pages 2 and 3 in flash bank 0
Flash Erase is verified

Attempting to partially erase pages 2 and 3 in flash bank 1
Flash Erase is verified

Example Succeeded
```
