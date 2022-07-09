## Description

This example configures the SPI to send data between the MISO (1.28) and MOSI (P1.29) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

The example can be configured to perform blocking (synchronous), non-blocking (asynchronus), or DMA transactions by defining/selecting the MASTERSYNC, MASTERASYNC, or MASTERDMA macro respectively.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P1.29 (MOSI) to P1.28 (MISO).

## Expected Output

The Console UART of the device will output these messages:

```
************** SPI Loopback Demo ****************
This example configures the SPI to send data between the MISO (P1.28) and
MOSI (P1.29) pins.  Connect these two pins together.  This demo shows SPI
sending different bit sizes each run through. If successful, the green LED will
illuminate.

This demo shows Asynchronous, Synchronous and DMA transaction for SPI1

--> 2 Bits Transaction Successful

--> 3 Bits Transaction Successful

--> 4 Bits Transaction Successful

--> 5 Bits Transaction Successful

--> 6 Bits Transaction Successful

--> 7 Bits Transaction Successful

--> 8 Bits Transaction Successful

--> 9 Bits Transaction Successful

-->10 Bits Transaction Successful

-->11 Bits Transaction Successful

-->12 Bits Transaction Successful

-->13 Bits Transaction Successful

-->14 Bits Transaction Successful

-->15 Bits Transaction Successful

-->16 Bits Transaction Successful
Example succeeded!
```
