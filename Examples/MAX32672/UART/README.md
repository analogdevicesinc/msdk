## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.28 to P0.15 (AIN7).
-   Connect P0.29 to P0.14 (AIN6).

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

The green LED (P0_23) will illuminate for successful transaction.
The red LED (P0_22) will illuminate if transaction failed.


Connect UART1 to UART2 for this example.
P0.28 -> P0.15 (AIN7) and P0.29 -> P0.14 (AIN6)


-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes

-->UART 1 Initialization Errors: 0
-->UART 2 Initialization Errors: 0

-->UART Initialized

-->Data verified

-->EXAMPLE SUCCEEDED

```

The green LED (P0_23) will illuminate for successful transaction.
The red LED (P0_22) will illuminate if transaction failed.
