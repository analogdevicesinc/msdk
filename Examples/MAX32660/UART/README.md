## Description

This application uses two serial ports to send and receive data.  One serial port transmits data while the other receives it. Supports DMA mode.

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P0.04 (UART0 TX) to P0.11 (UART1 RX).

## Expected Output

The Console UART of the device will output these messages:

```
**************** UART Example ******************
This example shows a loopback test between the 2 UARTs on the MAX32660.

Connect UART0 to UART1 (P0.4 -> P0.11) for this example.
The LEDs are used to indicate the success of the test.
Blinking->Success, Solid->Failure

-->UART Baud    : 115200 Hz

-->Test Length  : 1024 bytes

-->UART Initialized

-->Data verified

-->EXAMPLE SUCCEEDED
```

