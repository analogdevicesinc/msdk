## Description

This example demonstrates a SPI transaction between two distinct SPI peripherals on the MAX78002. 

SPI1 is setup as the controller (L. Master) in this example and is configured by default to send/receive 1024 8-bit words to and from the slave. Likewise, SPI0 is setup as the slave and is also expecting to both send and receive 1024 8-bit words to and from the master.

Once the controller ends the transaction, the data received by the controller (L. Master) and the target (L. Slave) is compared to the data sent by their counterpart to ensure all bytes were received properly.

This example also demonstrates the three available Target Select (TS) control schemes for the Controller which allow users to set up custom TS pins. By default, the example is set to `TSCONTROL_HW_AUTO`. The hardware automatically asserts/deasserts the pre-set TS pins. To use a custom TS pin where the SPI v2 Driver handles the TS assertions, reset the `TSCONTROL_HW_AUTO` macro to 0 and set the `TSCONTROL_SW_DRV` macro to 1.  To let the Application handle the TS assertions, set the `TSCONTROL_SW_APP` macro to 1 instead.


Target Select (CS) Pin Connections
- TSCONTROL_HW_AUTO: Connect (P0.4 to P0.20).
- TSCONTROL_SW_DRV and TSCONTROL_SW_APP: Connect (P0.4 to P0.12).

## Software

This project uses the SPI v2 library. More information on the SPI v2 library can be found in the **[MSDK User Guide Developer Notes](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#spi-v2-library)**.

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

Set `MXC_SPI_VERSION=v2` to build the SPI v2 libraries.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect the SPI pins on headers JH6 and JH9. (P0.4-->P0.20 (CS), P0.5-->P0.21 (MOSI), P0.6-->P0.22 (MISO), and P0.7-->P0.23 (SCK))
-   If custom target select pin was selected, re-connect the CS pins (P0.4-->P0.12).

## Expected Output

The Console UART of the device will output these messages:

```
************************ SPI Controller-Target Example ************************
This example sends data between two SPI peripherals in the MAX78002.
SPI1 is configured as the target (L. Slave) and SPI0 is configured
as the controller (L. Master). Each SPI peripheral sends 1024 bytes
on the SPI bus. If the data received by each SPI instance matches the
the data sent by the other instance, then the green LED will illuminate,
otherwise the red LED will illuminate.

Press PB1 to begin transaction.

Example Succeeded
```
