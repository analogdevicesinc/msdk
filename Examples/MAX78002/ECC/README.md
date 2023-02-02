## Description

The ECC engine is used to watch for errors in data RAM.  The memory is scanned to ensure no errors exist.  

A word is then modified to have a single bit error.  That word is read and the ECC flags are examined to make sure a correctable error is detected.  The expected behavior here is that the read of the corropted memory will result in the non-corrupted value, but the ECC error and ECC not double bit error flags will be set.

That same word is modified again to contain a double bit error.  The word is read and the ECC flags are examined to make sure a non-correctable error is reported.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

- For a "quick-start" or for first-time users see ["Getting Started"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#getting-started)
- See ["Development Guide"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#development-guide) for a detailed reference.

### Project-Specific Build Notes

(None)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX78002 SRAM ECC Example *****

This example will corrupt a word of data
and ensure that the ECC interrupts on an error
when the corrupted address is read

Preliminary scan to ensure no pre-existing ECC errors
536975769
PASS: No errors

Data before Corruption: 0xdeadbeef
Address of data: 0x20000e24

Data after single-bit error: 0xdeadbeef
ECC Error:              0x00000001
ECC Not Double Error:   0x00000001
ECC Error Address:      0x20000e24
PASS: An ECC Error was found

Data after double-bit error: 0xdeadbeec
ECC Error:              0x00000001
ECC Not Double Error:   0x00000000
ECC Error Address:      0x20000e24
PASS: An ECC Error was found

# Passed: 3, # Failed: 0, Test Ok
Example Complete
```

