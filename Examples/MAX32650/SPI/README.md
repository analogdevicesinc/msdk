## Description

This example configures the SPI to send data between the MISO and MOSI pins of
the specified SPI instance.  If no SPI instance is specified, the default instance
is SPI1.  Connect the MISO and MOSI lines together per the table below:

| SPI Inst | MISO | MOSI |
| -------- | ---- | ---- |
| SPI0     | 3.1  | 3.2  |
| SPI1     | 1.28 | 1.29 |
| SPI2     | 2.3  | 2.4  |
| SPI3     | 0.20 | 0.21 |

Multiple word sizes (2 through 16 bits) are demonstrated.

By default, the example performs blocking SPI transactions.  To switch to
non-blocking (asynchronous) transactions, reset the MASTERSYNC macro to 0 and set
the MASTERASYNC macro to 1.  To use DMA transactions, set the MASTERDMA macro to
1 instead.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

Update the project.mk file to specify the desired SPI instance via the TEST_SPI_NUM
definition. Valid values for TEST_SPI_NUM are 0-3:

`PROJ_CFLAGS += -DTEST_SPI_NUM=1`

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect MOSI to MISO per the desired SPI instance.

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
