## Description

This example demonstrates various functions of the MAX32655's flash controller, including flash mass erase, flash page erase and wiriting data to flash.

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the EV kit you are using to run the example.

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
