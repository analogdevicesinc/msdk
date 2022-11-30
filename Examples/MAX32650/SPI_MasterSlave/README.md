## Description

This example demonstrates a SPI transaction between two distinct SPI peripherals on the MAX32650. 

SPI2 is setup as the master in this example and is configured by default to send/receive 1024 8-bit words to and from the slave. Likewise, SPI1 is setup as the slave and is also expecting to both send and receive 1024 8-bit words to and from the master.

Once the master ends the transaction, the data received by the master and the slave is compared to the data sent by their counterpart to ensure all bytes were received properly.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect the SPI pins on headers JH3 and JH4. (P2.4-->P1.29 (MOSI), P2.3-->P1.28 (MISO), P2.2-->P1.26 (SCK), and P2.5-->P1.23 (CS))

## Expected Output

The Console UART of the device will output these messages:

```
************************ SPI Master-Slave Example ************************
This example sends data between two SPI peripherals in the MAX32650.
SPI1 is configured as the slave and SPI2 is configured as the master.
Each SPI peripheral sends 1024 bytes on the SPI bus. If the data received
by each SPI instance matches the data sent by the other instance, the
green LED will illuminate, otherwise the red LED will illuminate.

Press SW2 to begin transaction.

EXAMPLE SUCCEEDED!
```