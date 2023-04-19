## Description

This example configures the SPI to send data between the MISO (P2.27) and
MOSI (P2.28) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

By default, the example performs blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, undefine the MASTERSYNC macro and define the MASTERASYNC macro.  To use DMA transactions, define the MASTERDMA macro instead.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP4(RX_SEL) and JP5(TX_SEL) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pins P2.27->P2.28 together.

## Expected Output

The Console UART of the device will output these messages:

```
**************************** SPI MASTER TEST *************************
This example configures the SPI to send data between the MISO (P2.27) and
MOSI (P2.28) pins. Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

Performing blocking (synchronous) transactions...
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

Example Complete.
```
