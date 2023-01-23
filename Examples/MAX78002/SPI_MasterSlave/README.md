## Description

This example demonstrates a SPI transaction between two distinct SPI peripherals on the MAX78002. 

SPI1 is setup as the master in this example and is configured by default to send/receive 1024 8-bit words to and from the slave. Likewise, SPI0 is setup as the slave and is also expecting to both send and receive 1024 8-bit words to and from the master.

Once the master ends the transaction, the data received by the master and the slave is compared to the data sent by their counterpart to ensure all bytes were received properly.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect the SPI pins on headers JH6 and JH9. (P0.4-->P0.20 (CS), P0.5-->P0.21 (MOSI), P0.6-->P0.22 (MISO), and P0.7-->P0.23 (SCK))

## Expected Output

The Console UART of the device will output these messages:

```
************************ SPI Master-Slave Example ************************
This example sends data between two SPI peripherals in the MAX78002.
SPI1 is configured as the slave and SPI0 is configured as the master.
Each SPI peripheral sends 1024 bytes on the SPI bus. If the data received
by each SPI instance matches the data sent by the other instance, the
green LED will illuminate, otherwise the red LED will illuminate.

Press PB1 to begin transaction.

Example Succeeded
```