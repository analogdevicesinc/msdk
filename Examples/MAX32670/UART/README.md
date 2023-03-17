## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.28 to P0.15.
-   Connect P0.29 to P0.14.

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example sends data from one UART to another

The green LED (P0_23) will illuminate for successful transaction.
The red LED (P0_22) will illuminate if transaction failed.


Connect UART1 to UART2 for this example.
P0.28 -> P0.15 and P0.29 -> P0.14


-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes

-->UART Initialized

-->Data verified

-->Example Succeeded
```

The green LED (P0_23) will illuminate for successful transaction.
The red LED (P0_22) will illuminate if transaction failed.
