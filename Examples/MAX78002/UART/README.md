## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.12 to P1.1.
-   Connect P0.13 to P1.0.

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another


Connect UART1 to UART2 for this example.
P0.12 -> P1.1 and P0.13 -> P1.0


-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes

-->UART 1 Baud rate: 115384
-->UART 2 Baud rate: 115384

-->UART Initialized

-->Data verified

-->EXAMPLE SUCCEEDED
```

