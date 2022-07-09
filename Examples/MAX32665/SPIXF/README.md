## Description

This example communicates with the MX25 flash on the EvKit. It loads code onto it and then executes that code using the SPIX execute-in-place peripheral.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************* SPIX Example *********************
This example communicates with an MX25 flash on the EvKit
loads code onto it and then executes that code using the
SPIX execute-in-place peripheral

SPI Clock: 4000000 Hz

MX25 Initialized.

MX25 ID verified

Erasing first 64k sector
Erased

Quad mode enabled

Programming function (104 bytes @ 0x1000877c) into external MX25 flash
Programmed

Verifying external flash
Verified

Jumping to external flash (@ 0x08000001), watch for blinking LED.

Running code from external flash
Returned from external flash

Example Succeeded

```

