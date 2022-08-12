## Description

This example configures the SPI to send data between the MISO (P0.17) and MOSI (P0.18) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

By default, the example performs blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, undefine the MASTERSYNC macro and define the MASTERASYNC macro.  To use DMA transactions, define the MASTERDMA macro instead.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect AIN1/P0.17 (MOSI) to AIN2/P0.18 (MOSI).

## Expected Output

The Console UART of the device will output these messages:

```
************** SPI Loopback Demo ****************
This example configures the SPI to send data between the MISO (AIN2/P0.18) and
MOSI (AIN1/P0.17) pins.  Connect these two pins together.  This demo shows SPI
sending different bit sizes each run through.  If successful, the LED1(green or blue) will
illuminate.  If fails, the LED0(red) will illuminate.

Note: some board versions of the MAX32665 won't print out the transaction
status because the SPI and Console UART pins are shared.  Check the LEDs, as
stated above, for the example status.

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
```

The green/blue LED turns on after a successful transaction and the red LED turns on after a failed transaction.  On some MAX32665 board versions, the transaction status won't be printed on the terminal because the SPI and UART pins are shared.
