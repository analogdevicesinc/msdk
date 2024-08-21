## Description

This example demonstrates a series of blocking and non-blocking read/writes in 1-bit and 4-bit data bus modes with the inserted Micro SD Card.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
***** MAX78002 SDHC Example *****
Waiting for card.
Card inserted.
Card Initialized.
Card type: SDHC
SD clock ratio (at card) 2:1
--> 1-bit data bus example <--
blocking read/write ok
Passed blocking
blocking erase ok
blocking erase read ok
Passed erase
non-blocking write ok
non-blocking read ok
Passed async
--> 4-bit data bus example <--
blocking read/write ok
Passed blocking
blocking erase ok
blocking erase read ok
Passed erase
non-blocking write ok
non-blocking read ok
Passed async
--> Blocking, 4-bit data bus, multi-block example <--
 PASS
 *** END OF EXAMPLE ***

```
