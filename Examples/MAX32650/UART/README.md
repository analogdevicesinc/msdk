## Description

To demonstrate the use of the UART peripheral, data is sent between two UART ports on the MAX32650.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect UART1 RX (P2.14) to UART2 TX (P1.10).

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

Connect RX(P2.14) of UART1 and TX(P1.10) of UART2.

To indicate a successful UART transfer, LED1 will illuminate.

Push SW2 to continue

UART Baud       : 115200 Hz
Test Length     : 512 bytes

-->UART Initialized

-->Data verified

-->EXAMPLE SUCCEEDED
```
