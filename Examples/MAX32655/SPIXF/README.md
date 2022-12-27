## Description

This example communicates with the W25 flash on the EvKit. It loads "Analog Devices" array onto it and then reads it back.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************* SPIX Example *********************
This example communicates with an W25 flash on the EvKit
loads code onto it and then executes that code using the
SPIX execute-in-place peripheral

SPI Clock: 5000000 Hz

External flash Initialized.

External flash ID verified

Erasing first 64k sector
Erased

Quad mode enabled

Programming function (15 bytes @ 0x20000000) into external flash
Programmed

Verifying external flash
Verified

Example Succeeded

```

