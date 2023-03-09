## Description

This is a SPI loopback demonstration. Data is sent between the MOSI and MISO pins of the same SPI instance and after each transaction the data sent is compared to the data received. This test is performed several times with a different word size for each test.

By default, the example performs a blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, undefine the MASTERSYNC macro and define the MASTERASYNC macro.  To use DMA transactions, define the MASTERDMA macro instead.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Setup

If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Connect pins J4.14 (MOSI - P0.21) and J4.15 (MISO - P0.15)
-   Select "CAM" with jumper JP22.

If using the MAX78000FTHR (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Connect pins J8.12 (MOSI - P0.5) and J8.13 (MISO - P0.6)

## Expected Output

The Console UART of the device will output these messages:

```
**************************** SPI MASTER TEST *************************
This example configures SPI to send data between the MISO and MOSI
pins (pins 14 and 15 on header J4). Connect these two pins together.
Additionally, ensure that jumper JP22 is set to "CAM".

Multiple word sizes (2 through 16 bits) are demonstrated.

Performing blocking (synchronous) transactions...
--> 2 Bits Transaction Successful
--> 3 Bits Transaction Successful
--> 4 Bits Transaction Successful
--> 5 Bits Transaction Successful
--> 6 Bits Transaction Successful
--> 7 Bits Transaction Successful
--> 8 Bits Transaction Successful
Hardware does not support 9-bit wide characters.
-->10 Bits Transaction Successful
-->11 Bits Transaction Successful
-->12 Bits Transaction Successful
-->13 Bits Transaction Successful
-->14 Bits Transaction Successful
-->15 Bits Transaction Successful
-->16 Bits Transaction Successful

Example Complete.
```
