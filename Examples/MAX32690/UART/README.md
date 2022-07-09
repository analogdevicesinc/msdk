## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the board version you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP8(TX_EN) header.
-   Remove JP7(RX_EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pins P2.12->P1.9.

If using the Featherboard (FTHR\_Apps\_P1):
-   Only one UART instance available. Example not supported.

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

The green LED will illuminate for successful transaction.
The red LED will illuminate if transaction failed.

Remove JP7(RX_EN) header.
Connect UART0 to UART2 (P2.12->P1.9) for this example.


-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes
-->UART Initialized

-->Data verified

-->EXAMPLE SUCCEEDED
```
