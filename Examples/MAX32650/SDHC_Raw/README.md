## Description

This example demonstrates reading and writing to an SD card using both 1-bit and 4-bit data buses and using both blocking and non-blocking methods.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Insert a MicroSD card into EV Kit's SDHC slot.

## Expected Output

```
***** 32650 SDHC Example *****
Waiting for card.
Card inserted.
Card Initialized.
Card type: SDHC
SD clock ratio (at card) 4:1
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

