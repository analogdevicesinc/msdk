## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect (P2.6) to (P1.1).
-   Connect (P2.7) to (P1.0).

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

The green LED (P0_25) will illuminate for successful transaction.
The red LED (P0_24) will illuminate if transaction failed.


Connect UART3 to UART2 for this example.
P2.6 -> P1.1 and P2.7 -> P1.0


-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes
-->UART Initialized

-->Data verified

-->Example Succeeded
```

The green LED (P0_25) will illuminate for successful transaction.
The red LED (P0_24) will illuminate if transaction failed.
