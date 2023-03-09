## Description

This example configures the SPI to send data between the MISO (P0.14 - AIN6) and MOSI (P0.15 - AIN7) pins.  Connect these two pins together.

The SPI transaction is repeated for SPI character sizes 2 to 16.

By default, the example performs blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, undefine the MASTERSYNC macro and define the MASTERASYNC macro.  To use DMA transactions, define the MASTERDMA macro instead.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pins labeled P0.14 (AIN6) and P0.15 (AIN7) together.

## Expected Output

The Console UART of the device will output these messages:

```
**************************** SPI MASTER TEST *************************
This example configures the SPI to send data between the MISO (P0.14 - AIN6) and
MOSI (P0.15 - AIN7) pins.  Connect these two pins together.  This demo shows SPI
sending different bit sizes each run through.

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
