## Description

This example configures the SPI to send data between the MISO (P0.4) and
MOSI (P0.2) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

By default, the example performs blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, reset the MASTERSYNC macro to 0 and set the MASTERASYNC macro to 1.  To use DMA transactions, set the MASTERDMA macro to 1 instead.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the J1 (PWR-OBD-UART0) connector.
-   Connect pins JP19 (OBD VCOM EN) RX and TX header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
**************************** SPI MASTER TEST *************************
This example configures the SPI to send data between the MISO (P0.4) and
MOSI (P0.2) pins.  Connect these two pins together.

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
