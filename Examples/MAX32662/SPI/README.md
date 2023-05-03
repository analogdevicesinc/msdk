## Description

This example configures the SPI to send data between the MISO (P0.2) and
MOSI (P0.3) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

This example can demonstrate blocking, non-blocking, and DMA transactions methods controlled by the definitiona at the top of main. For blocking transactions define MASTERSYNC, for non-blocking transactions define MASTERASYNC, and for DMA transactions define MASTERDMA; define only one of these at a time. By default MASTERSYNC is defined.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pins labeled 2 and 3 on J1 (PORT 0) together.

## Expected Output

The Console UART of the device will output these messages:

```
**************************** SPI MASTER TEST *************************
This example configures the SPI to send data between the MISO (P0.2) and
MOSI (P0.3) pins.  Connect these two pins together.  This demo shows SPI
sending different bit sizes each run through.

This demo can be configured to show Asynchronous, Synchronous or DMA
transactions for SPI0.
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
